const values={provider:"openai",model:"test",apiKey:"",scenario:"1",battery:"92",saveDirectory:"session_logs",mission:"Find the red package, inspect it from three viewpoints, pick it up, fly through the gate, place it on the blue platform, and return home."};
const elements=new Map();
function element(id){if(!elements.has(id))elements.set(id,{value:values[id]||"",checked:true,disabled:false,hidden:true,innerHTML:"",textContent:"",className:"",width:900,height:520,appendChild(){},getContext(){return {}},classList:{add(){},remove(){}}});return elements.get(id)}
const document={getElementById:element,querySelectorAll:()=>[],createElement:()=>({innerHTML:"",textContent:"",className:""})};
const performance={now:()=>Date.now()};const location={protocol:"http:"};
function requestAnimationFrame(){};function alert(message){throw new Error(message)}
let timeoutCallback;
function setTimeout(callback){timeoutCallback=callback;return 1}function clearTimeout(){timeoutCallback=null}
