"""Constrained, memory-sequence behavior trees compiled from validated missions."""
import copy

def build_tree(spec,plan):
    policy=next((p for p in spec.get('contingencies',[]) if p.get('trigger')=='gate_not_clear'),{})
    def action(a):return {'type':'Action','skill':a['skill'],'args':list(a['args'])}
    children=[]
    for i,a in enumerate(plan):
        if a['skill']=='TRAVERSE_GATE':
            suffix=copy.deepcopy(plan[i+1:])
            branches=[{'type':'Sequence','children':[{'type':'Condition','predicate':'gate_clear','args':['gate_1']},action(a)]+[action(x) for x in suffix]}]
            if policy.get('action')=='BYPASS_GATE' and policy.get('authorization')=='USER':
                destination=policy.get('destination','blue_platform')
                alternate=[]
                for x in suffix:
                    if x['skill']=='NAVIGATE' and x['args'] in [['blue_platform'],['red_platform']]:x['args']=[destination]
                    if x['skill']=='PLACE':x['args']=['red_package',destination]
                    alternate.append(action(x))
                branches.append({'type':'Sequence','children':[{'type':'Condition','predicate':'gate_blocked','args':['gate_1']},action({'skill':'BYPASS_GATE','args':['gate_1']})]+alternate})
            children.append({'type':'Fallback','children':branches})
            break
        else:children.append(action(a))
    return {'format':'behavior_tree_v1','root':{'type':'Sequence','children':children}}

def nominal_actions(tree):
    if not isinstance(tree,dict) or tree.get('format')!='behavior_tree_v1':raise ValueError('Unsupported behavior-tree format')
    count=[0]
    def walk(n,depth=0):
        count[0]+=1
        if depth>8 or count[0]>400 or not isinstance(n,dict):raise ValueError('Invalid/oversized behavior tree')
        t=n.get('type')
        if t=='Action':return [{'skill':n.get('skill'),'args':n.get('args')}]
        if t=='Condition':return []
        children=n.get('children')
        if t not in {'Sequence','Fallback'} or not isinstance(children,list) or not children:raise ValueError('Invalid composite node')
        # Validate every branch's structure, even though only the first is nominal.
        rows=[walk(c,depth+1) for c in children]
        return rows[0] if t=='Fallback' else [a for row in rows for a in row]
    walk(tree.get('root'))
    actions=[]
    def nominal(n):
        if n['type']=='Action':actions.append({'skill':n['skill'],'args':n['args']});return True
        if n['type']=='Condition':return n.get('predicate')=='gate_clear'
        if n['type']=='Sequence':return all(nominal(c) for c in n['children'])
        return any(nominal(c) for c in n['children'])
    if not nominal(tree['root']):raise ValueError('No successful nominal branch')
    return actions

def execute_tree(tree,action,condition,event=lambda *a,**k:None):
    """Blocking memory sequence: successful actions run once, not every poll.

    Only a false condition selects a fallback. Exceptions from skills propagate
    to safe recovery, never masquerading as gate blockage or an alternate success.
    """
    def tick(node,path):
        event('BT_NODE',node_path=path,node_type=node['type'],status='RUNNING')
        t=node['type']
        if t=='Action':action({'skill':node['skill'],'args':node['args']});ok=True
        elif t=='Condition':ok=condition(node['predicate'])
        elif t=='Sequence':ok=all(tick(n,path+'.'+str(i)) for i,n in enumerate(node['children']))
        elif t=='Fallback':ok=any(tick(n,path+'.'+str(i)) for i,n in enumerate(node['children']))
        else:raise RuntimeError('UNSUPPORTED_BT_NODE')
        event('BT_NODE',node_path=path,node_type=t,status='SUCCESS' if ok else 'FAILURE');return ok
    return tick(tree['root'],'root')


def validate_generated_tree(tree,spec,available_platforms=('blue_platform','red_platform')):
    """Validate structure and symbolic goal coverage on both gate states.

    This checks a proposal; it never synthesizes/reorders/replaces the supplied tree.
    Node shapes and skill semantics are the controller contract, not a plan template.
    """
    nominal_actions(tree)
    policy=next((p for p in spec.get('contingencies',[]) if p.get('trigger')=='gate_not_clear'),{})
    destination=policy.get('destination','blue_platform')
    if destination not in available_platforms:raise ValueError('Alternate destination is unavailable in this scene')
    goals=spec.get('goals',[])
    views=[g['args'][1] for g in goals if g['predicate']=='inspected']
    action_skills={'SEARCH','INSPECT','PICK','NAVIGATE','TRAVERSE_GATE','PLACE','RETURN_HOME','HOVER'}
    nodes=[0]
    def shape(n,depth=0):
        nodes[0]+=1
        if nodes[0]>400 or depth>8 or not isinstance(n,dict):raise ValueError('Oversized behavior tree')
        kind=n.get('type')
        fields={'Sequence':{'type','children'},'Fallback':{'type','children'},'Condition':{'type','predicate','args'},'Action':{'type','skill','args'}}
        if kind not in fields or set(n)!=fields[kind]:raise ValueError('Invalid behavior-tree node fields')
        if kind in ['Sequence','Fallback']:
            if not isinstance(n['children'],list) or not n['children']:raise ValueError('Empty composite')
            for c in n['children']:shape(c,depth+1)
        elif kind=='Condition':
            if n['predicate'] not in ['gate_clear','gate_blocked'] or n['args']!=['gate_1']:raise ValueError('Unsupported condition')
        else:
            k=n['skill'];a=n['args']
            valid=(k in ['RETURN_HOME','HOVER'] and a==[] or k in ['SEARCH','PICK'] and a==['red_package'] or k=='INSPECT' and a and a[0]=='red_package' and len(a)==2 and a[1] in views or k=='NAVIGATE' and a in [[x] for x in ['target_loc','gate_loc','home',*available_platforms]] or k=='TRAVERSE_GATE' and a==['gate_1'] or k=='PLACE' and a in [['red_package',x] for x in available_platforms] or k=='BYPASS_GATE' and a==['gate_1'] and policy.get('action')=='BYPASS_GATE' and policy.get('authorization')=='USER')
            if not valid:raise ValueError('Unsupported or unauthorized behavior-tree action')
    shape(tree['root'])
    for clear in [True,False]:
        # A policy requiring a human decision is handled by existing recovery.
        if not clear and policy.get('action')!='BYPASS_GATE':continue
        state={'found':False,'views':set(),'holding':False,'picked':False,'traversed':False,'placed_on':None,'home':False,'bypass':False,'location':None};milestones={};counter=[0]
        def action(a):
            k=a['skill'];args=a['args'];counter[0]+=1
            if k=='SEARCH':state['found']=True
            elif k=='INSPECT':
                if not state['found']:raise ValueError('INSPECT before finding package')
                state['views'].add(args[1])
            elif k=='PICK':
                if not state['found'] or state['holding']:raise ValueError('Invalid PICK preconditions')
                state.update(holding=True,picked=True)
            elif k=='NAVIGATE':state['location']=args[0]
            elif k=='TRAVERSE_GATE':
                if not clear:raise ValueError('Tree traverses a blocked gate')
                state['traversed']=True
            elif k=='BYPASS_GATE':
                if clear or not state['holding']:raise ValueError('Invalid gate alternate preconditions')
                state['bypass']=True
            elif k=='PLACE':
                if not state['holding'] or state['location']!=args[1]:raise ValueError('PLACE lacks payload or destination navigation')
                state.update(holding=False,placed_on=args[1])
            elif k=='RETURN_HOME':state['home']=True
            for g in goals:
                p=g['predicate'];a=g['args']
                ok={'found':lambda:state['found'],'inspected':lambda:a[1] in state['views'],'holding':lambda:state['picked'],'traversed':lambda:state['traversed'] or state['bypass'],'placed':lambda:state['placed_on']==(destination if state['bypass'] else a[1]),'at':lambda:state['home']}.get(p,lambda:False)()
                if ok:milestones.setdefault(g['id'],counter[0])
        ok=execute_tree(tree,action,lambda p:clear if p=='gate_clear' else not clear)
        if not ok or any(g['id'] not in milestones for g in goals):raise ValueError('Behavior tree omits a required goal on a reachable branch')
        if state['holding'] and any(g['predicate']=='placed' for g in goals):raise ValueError('Tree ends carrying an undelivered package')
        for edge in spec.get('ordering',[]):
            if milestones[edge['before']]>=milestones[edge['after']]:raise ValueError('Behavior tree violates goal ordering')
    return tree
