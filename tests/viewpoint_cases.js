const moved=[];
move=async function(dest){planMotion(world.drone,dest);moved.push(dest);world.drone.x=dest.x;world.drone.y=dest.y};
function assert(ok,msg){if(!ok)throw new Error(msg)}

async function test(){
for(const count of [1,2,3,4]){
 const text=`Find the red package, inspect it from ${count} viewpoints, pick it up, fly through the gate, place it on the blue platform, and return home.`;
 missionSpec=localCompile(text);const ins=inspectionGoals(missionSpec),plan=buildPlan(missionSpec),pddl=compilePddl(missionSpec);
 assert(ins.length===count,"Compiler lost viewpoints");assert(plan.ok,"Plan rejected");
 const actions=plan.plan.filter(a=>a.skill==="INSPECT");assert(actions.length===count,"Plan lost viewpoints");
 for(const g of ins){assert(pddl.includes(`(inspected red_package ${g.args[1]})`),"PDDL lost viewpoint");assert(actions.some(a=>JSON.stringify(a.args)===JSON.stringify(g.args)),"Plan changed goal arguments")}
 const holding=missionSpec.goals.find(g=>g.predicate==="holding");for(const g of ins)assert(missionSpec.ordering.some(o=>o.before===g.id&&o.after===holding.id),"Pickup precedes inspection");
 moved.length=0;verifiedState.inspectedViews=[];
 for(const a of actions)await skill(a);
 assert(new Set(moved.map(p=>JSON.stringify(p))).size===count,"Repeated physical viewpoint");
 assert(verifiedState.inspectedViews.length===count,"Execution lost inspections");
}
missionSpec=localCompile("Find the red package, inspect it from three viewpoints, and return home.");
assert(inspectionGoals(missionSpec).length===3,"Word count failed");
missionSpec.goals=missionSpec.goals.filter(g=>g.args[1]!=="view_c");assert(!buildPlan(missionSpec).ok,"Missing inspection silently accepted");
let failed=false;try{compilePddl(missionSpec)}catch(e){failed=true}assert(failed,"PDDL accepted missing inspection");
missionSpec=localCompile("Inspect the red package from three viewpoints.");missionSpec.goals[2].args=[...missionSpec.goals[1].args];assert(!buildPlan(missionSpec).ok,"Duplicate viewpoint accepted");
print("PASS: 1–4 viewpoints preserved in compiler, PDDL, plan, ordering and simulated inspection execution; missing/duplicate viewpoints rejected.");
}
test().catch(e=>{printerr(e.stack);imports.system.exit(1)});
