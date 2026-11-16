// A small DOM for testing actual chat handlers and card actions.
function enhance(n){
 n.children=n.children||[];n.dataset={};n.appendChild=function(c){this.children.push(c);return c};n.removeChild=function(c){this.children.splice(this.children.indexOf(c),1)};
 Object.defineProperty(n,'firstElementChild',{get(){return this.children[0]},configurable:true});
 n.setAttribute=function(k,v){this[k]=v};n.focus=function(){};n.getContext=()=>({});return n;
}
for(const n of elements.values())enhance(n);
const originalElement=element;document.getElementById=id=>{const n=originalElement(id);return n.children?n:enhance(n)};
document.createElement=tag=>enhance({tagName:tag,textContent:'',value:'',disabled:false,checked:false});
function descendants(n){return [n,...(n.children||[]).flatMap(descendants)];}
function action(label){return descendants(document.getElementById('chatThread')).filter(n=>n.tagName==='button'&&n.textContent===label).at(-1);}
