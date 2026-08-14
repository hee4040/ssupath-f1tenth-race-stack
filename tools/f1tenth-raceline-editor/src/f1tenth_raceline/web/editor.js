(() => {
  const $ = id => document.getElementById(id);
  const base = $('base'), overlay = $('overlay');
  const bctx = base.getContext('2d', { alpha: false });
  const octx = overlay.getContext('2d');
  const stage = $('stage');
  let mode = 'map';
  let tool = 'free';
  let operations = [], redoStack = [], currentStroke = null;
  let centerline = [], showCenterline = true;
  let sectorData = null;
  let img = new Image();
  let previewTimer = null, validateTimer = null;
  const palette = ['#70a7ff','#d9a45f','#8bd17c','#cf83e8','#5fc8c8','#e98181','#c7c75e','#8f9ed9'];
  function mapProfile() { return { version: 1, operations }; }
  function currentSectorKey() { return mode === 'speed' ? 'speed' : 'overtaking'; }
  function setStatus(el, text, error=false) { el.textContent = text; el.style.color = error ? '#ff8d97' : '#aab4c0'; }
  function setMode(next) {
    if (next !== 'map' && !sectorData?.available) return;
    if (next === 'ot' && !sectorData?.shortest_available) return;
    mode = next;
    $('mapPanel').classList.toggle('hidden', mode !== 'map');
    $('sectorPanel').classList.toggle('hidden', mode === 'map');
    $('speedGlobals').classList.toggle('hidden', mode !== 'speed');
    $('otGlobals').classList.toggle('hidden', mode !== 'ot');
    $('mapModeBtn').classList.toggle('active', mode === 'map');
    $('speedModeBtn').classList.toggle('active', mode === 'speed');
    $('otModeBtn').classList.toggle('active', mode === 'ot');
    overlay.style.cursor = mode === 'map' ? 'crosshair' : 'cell';
    redoStack = [];
    if (mode !== 'map') renderSectorControls();
    redraw(); updateButtons();
  }
  function updateButtons() {
    $('undoBtn').disabled = mode !== 'map' || operations.length === 0;
    $('redoBtn').disabled = mode !== 'map' || redoStack.length === 0;
    $('freeBtn').classList.toggle('active', tool === 'free');
    $('occupiedBtn').classList.toggle('active', tool === 'occupied');
  }
  function drawStroke(ctx, op, alpha=0.52) {
    if (!op.points?.length) return;
    const color = op.tool === 'free' ? `rgba(74,163,255,${alpha})` : `rgba(255,91,105,${alpha})`;
    ctx.save(); ctx.strokeStyle=color; ctx.fillStyle=color; ctx.lineWidth=op.radius*2; ctx.lineCap='round'; ctx.lineJoin='round';
    ctx.beginPath(); ctx.moveTo(op.points[0][0], op.points[0][1]); for (const p of op.points.slice(1)) ctx.lineTo(p[0], p[1]); ctx.stroke();
    for (const p of [op.points[0], op.points[op.points.length-1]]) { ctx.beginPath(); ctx.arc(p[0],p[1],op.radius,0,Math.PI*2); ctx.fill(); }
    ctx.restore();
  }
  function sectorIndexAtS(s, splits) { let i=0; while (i < splits.length && s > splits[i]) i++; return i; }
  function drawPolyline(points, color, width=1.5, dashed=false) {
    if (!points || points.length < 2) return;
    octx.save(); octx.strokeStyle=color; octx.lineWidth=width; if (dashed) octx.setLineDash([7,5]);
    octx.beginPath(); octx.moveTo(points[0][1], points[0][2]); for (const p of points.slice(1)) octx.lineTo(p[1],p[2]); octx.stroke(); octx.restore();
  }
  function drawXYPolyline(points, color, width=1) {
    if (!points || points.length < 2) return;
    octx.save(); octx.strokeStyle=color; octx.lineWidth=width; octx.globalAlpha=.6;
    octx.beginPath(); octx.moveTo(points[0][0],points[0][1]); for(const p of points.slice(1)) octx.lineTo(p[0],p[1]); octx.stroke(); octx.restore();
  }
  function drawSectors() {
    if (!sectorData?.available) return;
    drawXYPolyline(sectorData.bounds?.right, '#5b6772', 1); drawXYPolyline(sectorData.bounds?.left, '#5b6772', 1);
    if (mode === 'ot' && sectorData.shortest?.length) drawPolyline(sectorData.shortest, '#ff6b6b', 1.3, true);
    const group = sectorData.profile[currentSectorKey()], splits = group.splits_s_m || [], points = sectorData.raceline || [];
    for (let i=1;i<points.length;i++) {
      const sec = sectorIndexAtS(points[i][0], splits); let color = palette[sec % palette.length], width = 3;
      if (mode === 'ot' && !group.sectors[sec]?.ot_flag) { color='#7f8994'; width=2; }
      octx.save(); octx.strokeStyle=color; octx.lineWidth=width; octx.beginPath(); octx.moveTo(points[i-1][1],points[i-1][2]); octx.lineTo(points[i][1],points[i][2]); octx.stroke(); octx.restore();
    }
    for (let i=0;i<splits.length;i++) {
      const split=splits[i]; let nearest=points[0]; for(const p of points) if(Math.abs(p[0]-split)<Math.abs(nearest[0]-split)) nearest=p;
      octx.save(); octx.fillStyle='#fff'; octx.strokeStyle='#101318'; octx.lineWidth=2; octx.beginPath(); octx.arc(nearest[1],nearest[2],5,0,Math.PI*2); octx.fill(); octx.stroke(); octx.fillStyle='#fff'; octx.font='12px system-ui'; octx.fillText(`S${i+1}`,nearest[1]+7,nearest[2]-7); octx.restore();
    }
  }
  function redraw() {
    octx.clearRect(0,0,overlay.width,overlay.height);
    if(mode==='map') {
      for(const op of operations) drawStroke(octx,op); if(currentStroke) drawStroke(octx,currentStroke,.7);
      if(showCenterline&&centerline.length>1){octx.save();octx.strokeStyle='#42e29c';octx.lineWidth=2;octx.beginPath();octx.moveTo(centerline[0][0],centerline[0][1]);for(const p of centerline.slice(1))octx.lineTo(p[0],p[1]);octx.closePath();octx.stroke();octx.restore();}
    } else drawSectors(); updateButtons();
  }
  function canvasPoint(evt){const rect=overlay.getBoundingClientRect();return[(evt.clientX-rect.left)*overlay.width/rect.width,(evt.clientY-rect.top)*overlay.height/rect.height];}
  function schedulePreview(){clearTimeout(previewTimer);previewTimer=setTimeout(previewCenterline,350);}
  async function previewCenterline(){
    if(mode!=='map')return; setStatus($('status'),'Recomputing centerline preview...'); $('previewBtn').disabled=true;
    try{const res=await fetch('/api/preview-centerline',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(mapProfile())});const data=await res.json();if(!data.ok)throw new Error(data.error||'preview failed');centerline=data.points||[];redraw();setStatus($('status'),`Centerline preview: ${centerline.length} points`);}catch(e){centerline=[];redraw();setStatus($('status'),`Preview failed: ${e.message}`,true);}finally{$('previewBtn').disabled=false;}
  }
  async function saveMap(){setStatus($('status'),'Saving edit profile...');try{const res=await fetch('/api/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(mapProfile())});const data=await res.json();if(!data.ok)throw new Error(data.error||'save failed');setStatus($('status'),`Saved ${data.operations} edit strokes.`);}catch(e){setStatus($('status'),`Save failed: ${e.message}`,true);}}
  function clone(v){return JSON.parse(JSON.stringify(v));}
  function addSplit(s){const key=currentSectorKey(),group=sectorData.profile[key];if(group.splits_s_m.some(v=>Math.abs(v-s)<1e-9))return;let pos=group.splits_s_m.findIndex(v=>v>s);if(pos<0)pos=group.splits_s_m.length;group.splits_s_m.splice(pos,0,s);const src=group.sectors[Math.min(pos,group.sectors.length-1)]||(key==='speed'?{scaling:.5,only_FTG:false,no_FTG:false}:{ot_flag:false});group.sectors.splice(pos+1,0,clone(src));renderSectorControls();redraw();scheduleValidate();}
  function removeSplit(index){const group=sectorData.profile[currentSectorKey()];group.splits_s_m.splice(index,1);if(group.sectors.length>1)group.sectors.splice(index+1,1);renderSectorControls();redraw();scheduleValidate();}
  function nearestRacelinePoint(x,y){let best=null,bestD=Infinity;for(const p of sectorData.raceline){const d=(p[1]-x)**2+(p[2]-y)**2;if(d<bestD){bestD=d;best=p;}}return bestD <= 35*35 ? best : null;}
  function sectorRangeText(group,i){const start=i===0?0:group.splits_s_m[i-1],end=i<group.splits_s_m.length?group.splits_s_m[i]:sectorData.s_max;return`${start.toFixed(2)} m → ${end.toFixed(2)} m`;}
  function renderWarnings(warnings){const box=$('warningList');box.innerHTML='';if(!warnings?.length){box.textContent='No validation warnings.';return;}for(const w of warnings){const d=document.createElement('div');d.className='warning'+(w.severity==='error'?' error':'');d.textContent=w.message;box.appendChild(d);}}
  function renderSectorControls(){
    if(!sectorData?.available)return;const key=currentSectorKey(),group=sectorData.profile[key];$('sectorTitle').textContent=mode==='speed'?'Speed sectors':'Overtaking sectors';
    if(mode==='speed')$('globalLimit').value=group.global_limit;else{$('yeetFactor').value=group.yeet_factor;$('splineLen').value=group.spline_len;$('otSectorBegin').value=group.ot_sector_begin;}
    const list=$('sectorList');list.innerHTML='';group.sectors.forEach((sec,i)=>{const card=document.createElement('div');card.className='sector-card';const title=document.createElement('div');title.className='title';const name=document.createElement('strong');name.textContent=(mode==='speed'?'Sector ':'OT Sector ')+i;name.style.color=palette[i%palette.length];title.appendChild(name);if(i>0){const del=document.createElement('button');del.textContent='Remove split';del.addEventListener('click',()=>removeSplit(i-1));title.appendChild(del);}card.appendChild(title);const meta=document.createElement('div');meta.className='meta';meta.textContent=sectorRangeText(group,i);card.appendChild(meta);
      if(mode==='speed'){const label=document.createElement('label');label.textContent='Velocity scaling';const inp=document.createElement('input');inp.type='number';inp.min='0';inp.max='1';inp.step='.01';inp.value=sec.scaling;inp.addEventListener('change',()=>{sec.scaling=Number(inp.value);scheduleValidate();redraw();});label.appendChild(inp);card.appendChild(label);const checks=document.createElement('div');checks.className='check-row';for(const field of['only_FTG','no_FTG']){const lab=document.createElement('label'),c=document.createElement('input');c.type='checkbox';c.checked=!!sec[field];c.addEventListener('change',()=>{sec[field]=c.checked;scheduleValidate();});lab.appendChild(c);lab.append(field);checks.appendChild(lab);}card.appendChild(checks);}else{const checks=document.createElement('div');checks.className='check-row';const lab=document.createElement('label'),c=document.createElement('input');c.type='checkbox';c.checked=!!sec.ot_flag;c.addEventListener('change',()=>{sec.ot_flag=c.checked;scheduleValidate();redraw();});lab.appendChild(c);lab.append('Enable shortest-path overtaking');checks.appendChild(lab);card.appendChild(checks);}list.appendChild(card);});renderWarnings(sectorData.warnings||[]);
  }
  async function validateSectors(){if(!sectorData?.available)return;try{const res=await fetch('/api/sectors/validate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(sectorData.profile)});const data=await res.json();if(!data.ok)throw new Error(data.error||'validation failed');sectorData.warnings=data.warnings||[];renderWarnings(sectorData.warnings);}catch(e){setStatus($('sectorStatus'),`Validation failed: ${e.message}`,true);}}
  function scheduleValidate(){clearTimeout(validateTimer);validateTimer=setTimeout(validateSectors,200);}
  async function saveSectors(){setStatus($('sectorStatus'),'Saving sectors and exporting ROS YAML...');try{const res=await fetch('/api/sectors/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(sectorData.profile)});const data=await res.json();if(!data.ok)throw new Error(data.error||'save failed');sectorData.profile=data.profile;sectorData.warnings=data.warnings||[];renderSectorControls();redraw();setStatus($('sectorStatus'),`Saved sectors.json\n${data.speed_yaml_path}\n${data.ot_yaml_path}`);}catch(e){setStatus($('sectorStatus'),`Sector save failed: ${e.message}`,true);}}
  async function loadSectors(){try{const data=await fetch('/api/sectors?t='+Date.now()).then(r=>r.json());sectorData=data;$('speedModeBtn').disabled=!data.available;$('otModeBtn').disabled=!data.available || !data.shortest_available;if(!data.available){setStatus($('sectorStatus'),data.message||'Generate racelines first.');if(mode!=='map')setMode('map');return;}if(!data.shortest_available)setStatus($('sectorStatus'),'Shortest-path raceline is missing; overtaking editor is disabled until raceline generation completes.');if(mode==='ot'&&!data.shortest_available){setMode('speed');setStatus($('sectorStatus'),'raceline_shortest.csv is missing; overtaking-sector editing is disabled.',true);}if(mode!=='map')renderSectorControls();redraw();}catch(e){sectorData={available:false};$('speedModeBtn').disabled=true;$('otModeBtn').disabled=true;setStatus($('sectorStatus'),`Sector load failed: ${e.message}`,true);}}
  overlay.addEventListener('pointerdown',evt=>{if(mode!=='map'){const[x,y]=canvasPoint(evt),p=nearestRacelinePoint(x,y);if(!p){setStatus($('sectorStatus'),'Click closer to the minimum-curvature raceline.',true);return;}if(p[3]<=0||p[3]>=sectorData.n_points-1){setStatus($('sectorStatus'),'Choose an interior waypoint, not the start/end sentinel.',true);return;}addSplit(p[0]);setStatus($('sectorStatus'),`Added split at s=${p[0].toFixed(2)} m, waypoint ${p[3]}.`);return;}overlay.setPointerCapture(evt.pointerId);currentStroke={tool,radius:Number($('radius').value),points:[canvasPoint(evt).map(Math.round)]};redoStack=[];redraw();});
  overlay.addEventListener('pointermove',evt=>{if(mode!=='map'||!currentStroke)return;const p=canvasPoint(evt).map(Math.round),last=currentStroke.points.at(-1);if(Math.hypot(p[0]-last[0],p[1]-last[1])>=1.5)currentStroke.points.push(p);redraw();});
  function finishStroke(evt){if(mode!=='map'||!currentStroke)return;currentStroke.points.push(canvasPoint(evt).map(Math.round));operations.push(currentStroke);currentStroke=null;centerline=[];redraw();schedulePreview();}
  overlay.addEventListener('pointerup',finishStroke);overlay.addEventListener('pointercancel',()=>{currentStroke=null;redraw();});
  $('mapModeBtn').addEventListener('click',()=>setMode('map'));$('speedModeBtn').addEventListener('click',()=>setMode('speed'));$('otModeBtn').addEventListener('click',()=>setMode('ot'));
  $('freeBtn').addEventListener('click',()=>{tool='free';updateButtons();});$('occupiedBtn').addEventListener('click',()=>{tool='occupied';updateButtons();});
  $('undoBtn').addEventListener('click',()=>{if(mode==='map'&&operations.length){redoStack.push(operations.pop());centerline=[];redraw();schedulePreview();}});$('redoBtn').addEventListener('click',()=>{if(mode==='map'&&redoStack.length){operations.push(redoStack.pop());centerline=[];redraw();schedulePreview();}});
  $('clearBtn').addEventListener('click',()=>{if(operations.length&&confirm('Clear all map edits?')){redoStack=[];operations=[];centerline=[];redraw();schedulePreview();}});$('radius').addEventListener('input',()=>$('radiusValue').textContent=$('radius').value);
  $('zoom').addEventListener('input',()=>{$('zoomValue').textContent=$('zoom').value;const scale=Number($('zoom').value)/100;stage.style.transform=`scale(${scale})`;stage.style.marginRight=`${stage.offsetWidth*(scale-1)}px`;stage.style.marginBottom=`${stage.offsetHeight*(scale-1)}px`;});
  $('previewBtn').addEventListener('click',previewCenterline);$('toggleCenterBtn').addEventListener('click',()=>{showCenterline=!showCenterline;$('toggleCenterBtn').textContent=showCenterline?'Hide centerline':'Show centerline';redraw();});$('saveBtn').addEventListener('click',saveMap);$('saveSectorsBtn').addEventListener('click',saveSectors);$('refreshSectorsBtn').addEventListener('click',loadSectors);
  $('globalLimit').addEventListener('change',()=>{sectorData.profile.speed.global_limit=Number($('globalLimit').value);scheduleValidate();});$('yeetFactor').addEventListener('change',()=>{sectorData.profile.overtaking.yeet_factor=Number($('yeetFactor').value);scheduleValidate();});$('splineLen').addEventListener('change',()=>{sectorData.profile.overtaking.spline_len=Number($('splineLen').value);scheduleValidate();});$('otSectorBegin').addEventListener('change',()=>{sectorData.profile.overtaking.ot_sector_begin=Number($('otSectorBegin').value);scheduleValidate();});
  window.addEventListener('keydown',evt=>{if((evt.ctrlKey||evt.metaKey)&&evt.key.toLowerCase()==='z'&&mode==='map'){evt.preventDefault();evt.shiftKey?$('redoBtn').click():$('undoBtn').click();}});
  fetch('/api/state').then(r=>r.json()).then(state=>{operations=state.operations||[];$('generateCommand').textContent=state.generate_command;img.onload=()=>{base.width=overlay.width=state.width;base.height=overlay.height=state.height;bctx.drawImage(img,0,0,state.width,state.height);redraw();previewCenterline();loadSectors();};img.src='/map.png?t='+Date.now();}).catch(e=>setStatus($('status'),`Initialization failed: ${e.message}`,true));
})();
