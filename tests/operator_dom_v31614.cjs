// Pure JavaScript unit harness. No browser, camera, network or robot is opened.
const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert/strict');
const html=fs.readFileSync(path.join(__dirname,'../web/operator.html'),'utf8');
const source=html.match(/<script>([\s\S]*?)<\/script>/)[1];
const nodes=new Map(),all=[],calls=[],intervals=[];let failCamera=false,next=0;
class Node {
 constructor(id='',data={}){this.id=id;this.dataset=data;this.disabled=false;this.checked=true;this.value='';this.className='';this.textContent='';this.attrs={};all.push(this);if(id)nodes.set(id,this);}
 get classList(){const n=this;return {add(c){n.className+=' '+c},remove(c){n.className=n.className.split(' ').filter(x=>x!==c).join(' ')},contains(c){return n.className.split(' ').includes(c)}}}
 setAttribute(k,v){this.attrs[k]=v}
 set innerHTML(v){this.child=new Node();this.child.dataset.manual='';for(const m of v.matchAll(/id="([^"]+)"/g))new Node(m[1]);}
 querySelector(){return this.child}
 append(){}
 click(){return this.onclick?.()}
 setPointerCapture(){}
 set src(v){this._src=v;queueMicrotask(()=>this.onload?.())}
 get src(){return this._src}
}
for(const m of html.matchAll(/\bid="([^"]+)"/g))if(!m[1].includes('${'))new Node(m[1]);
for(const m of html.matchAll(/<button\b([^>]+)>/g)){
 const attrs=m[1],id=attrs.match(/\bid="([^"]+)"/)?.[1];const n=id?nodes.get(id):new Node();
 for(const attr of ['command','drive']){const match=attrs.match(new RegExp('data-'+attr+'="([^"]+)"'));if(match)n.dataset[attr]=match[1];}
}
const state={connected:true,fresh:true,stopped:false,busy:false,epoch:7,auto_running:false,drive:{active:false},config:{grip:{open:600,close:2300}},hardware:{joints:{p1:1300,p10:950,p11:1250,d2:2100,d3:600,d4:2500},motors:[0,0,0,0]},auto:{mechanics_enabled:false,operator_protocol:2,wrist:{angle_deg:0},sensor_grasp_ready:true,sensor_min_mm:200,sensor_max_mm:250,message:'Sẵn sàng'},gift:{available:true,busy:false,message:'Sẵn sàng'},command_trace:[]};
const document={hidden:false,activeElement:null,fullscreenElement:null,getElementById(id){assert(nodes.has(id),'Missing HTML element '+id);return nodes.get(id)},createElement(){return new Node()},querySelectorAll(selector){return all.filter(n=>selector.split(',').some(s=>s==='[data-command]'?'command' in n.dataset:s==='[data-drive]'?'drive'in n.dataset:s==='[data-manual]'?'manual'in n.dataset:false))},addEventListener(){}};
const context={document,window:{},console,AbortSignal,Date,Promise,URL:{createObjectURL(){return 'blob:qa'+(++next)},revokeObjectURL(){}},setInterval(fn){intervals.push(fn)},setTimeout(){return 1},clearTimeout(){},fetch:async(url,options={})=>{
 calls.push({url,options});
 if(url.startsWith('/api/preview'))return failCamera?{ok:false,json:async()=>({detail:'Camera chưa có ảnh mới'})}:{ok:true,blob:async()=>({})};
 return {ok:true,json:async()=>structuredClone(state)};
}};
vm.createContext(context);vm.runInContext(source,context);
const settle=()=>new Promise(resolve=>setImmediate(resolve));
(async()=>{
 await settle();
 assert.equal(calls.filter(c=>c.url==='/operator/action').length,0,'Loading the panel must not move actuators');
 assert.equal(nodes.get('gripClosePulse').textContent,2300);
 assert(nodes.get('cameraViewport').classList.contains('live'));
 assert.equal(nodes.get('gift').disabled,false);
 nodes.get('gift').onclick();await settle();
 assert.equal(JSON.parse(calls.find(c=>c.url==='/operator/action').options.body).command,'GIFT DISPENSE');
 await nodes.get('cameraFullscreen').onclick();assert(nodes.get('cameraPanel').classList.contains('expanded'));
 await nodes.get('cameraFullscreen').onclick();assert(!nodes.get('cameraPanel').classList.contains('expanded'));
 nodes.get('cameraStop').onclick();await settle();
 assert(calls.some(c=>c.url==='/operator/action'&&JSON.parse(c.options.body).command==='STOP'));
 nodes.get('showGuides').checked=false;nodes.get('showGuides').onchange();await settle();
 assert(calls.some(c=>c.url.startsWith('/api/preview?guides=false')));
 failCamera=true;await vm.runInContext('frame()',context);
 assert(!nodes.get('cameraViewport').classList.contains('live'));
 assert.equal(nodes.get('cameraHelp').textContent,'Camera chưa có ảnh mới');
 failCamera=false;await vm.runInContext('frame()',context);assert(nodes.get('cameraViewport').classList.contains('live'));
 assert(html.includes('@media(max-width:900px)')&&html.includes('grid-template-columns:minmax(0,1.55fr) minmax(365px,1fr)'));
 console.log('PASS: initial load sends no actuator commands; 2300us display; shared camera; gift; fullscreen fallback; STOP; guides; camera loss/recovery; responsive CSS hooks.');
})().catch(e=>{console.error(e);process.exitCode=1});
