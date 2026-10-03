import json

TEMPLATE = r'''<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__ - rsf_bag_visualizer</title>
<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
<style>
:root{--page:#f9f9f7;--surface:#fcfcfb;--ink:#0b0b0b;--ink2:#52514e;--muted:#898781;--grid:#e1e0d9;--line:#c3c2b7;--blue:#2a78d6}
*{box-sizing:border-box}
body{margin:0;background:var(--page);color:var(--ink);font:13px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif}
header{display:flex;flex-wrap:wrap;align-items:baseline;gap:16px;padding:12px 16px 4px}
header h1{font-size:18px;margin:0;font-weight:600}
header span{color:var(--ink2)}
nav{display:flex;flex-wrap:wrap;gap:4px;padding:6px 16px;border-bottom:1px solid var(--grid)}
nav button{border:1px solid var(--line);background:var(--surface);color:var(--ink2);padding:6px 12px;border-radius:6px;cursor:pointer;font:inherit}
nav button.on{background:var(--blue);border-color:var(--blue);color:#fff}
#controls{display:flex;flex-wrap:wrap;align-items:center;gap:10px;padding:8px 16px;color:var(--ink2)}
#controls input[type=number]{width:90px;padding:3px 6px;border:1px solid var(--line);border-radius:4px;font:inherit}
#controls button{border:1px solid var(--line);background:var(--surface);padding:3px 10px;border-radius:4px;cursor:pointer;font:inherit}
main{padding:0 16px 24px}
.panel>div{background:var(--surface);border:1px solid var(--grid);border-radius:8px;margin-bottom:12px}
.htmlpanel{padding:12px 16px}
.tiles{display:flex;flex-wrap:wrap;gap:10px;margin-bottom:8px}
.tile{border:1px solid var(--grid);border-radius:8px;padding:8px 14px;min-width:150px}
.tile .k{color:var(--ink2);font-size:12px}
.tile .v{font-size:20px}
table{border-collapse:collapse;margin:4px 0 12px}
th,td{text-align:left;padding:4px 10px;border-bottom:1px solid var(--grid);font-variant-numeric:tabular-nums}
th{color:var(--ink2);font-weight:600}
tr.ev{cursor:pointer}
tr.ev:hover{background:#f0efec}
.sw{display:inline-block;width:14px;height:4px;border-radius:2px;margin-right:8px;vertical-align:middle}
h3{font-size:14px;margin:12px 0 4px}
.ptitle{font-size:14px;font-weight:600;padding:10px 16px 0}
</style>
</head>
<body>
<header><h1>__TITLE__</h1><span id="meta"></span></header>
<nav id="tabs"></nav>
<div id="controls">
<span>時間範囲</span><input id="t0" type="number" step="1"><span>–</span><input id="t1" type="number" step="1"><span>s</span>
<button id="reset">全体</button>
<label><input id="align" type="checkbox"> LIO を GNSS 基準に位置合わせ</label>
<span id="alignInfo"></span>
</div>
<main id="panels"></main>
<script>const D = __DATA__;</script>
<script>
const C={grid:'#e1e0d9',surface:'#fcfcfb',ink:'#0b0b0b',ink2:'#52514e',muted:'#898781',blue:'#2a78d6',orange:'#eb6834',aqua:'#1baf7a',violet:'#4a3aa7',red:'#e34948'};
const TABS=[['overview','概要'],['rsf','rsf_odom（切替）'],['lio_raw','LIO 生'],['lio_rsf','RSF 処理 LIO'],['yaw','方位 (yaw)'],['sim','理想切替'],['gnss','GNSS'],['states','状態'],['events','イベント・集計']];
const S={t0:0,t1:D.meta.duration_s,align:false,tab:'overview',dirty:{}};
const HOVER='t=%{customdata:.1f}s<br>x=%{x:.2f} y=%{y:.2f}<extra>%{fullData.name}</extra>';
function lower(a,v){let lo=0,hi=a.length;while(lo<hi){const m=(lo+hi)>>1;if(a[m]<v)lo=m+1;else hi=m;}return lo;}
function range(src){return [lower(src.t,S.t0),lower(src.t,S.t1)];}
function sl(t,v){const i=lower(t,S.t0),j=lower(t,S.t1);return {t:t.slice(i,j),v:v.slice(i,j)};}
function pts(src,native){const [i,j]=range(src);const a=!native&&S.align&&src.xa;return {t:src.t.slice(i,j),x:(a?src.xa:src.x).slice(i,j),y:(a?src.ya:src.y).slice(i,j)};}
function trackTrace(src,name,color,dash,width,native){const p=pts(src,native);return {type:'scattergl',mode:'lines',name,x:p.x,y:p.y,customdata:p.t,line:{color,dash,width:width||2},hovertemplate:HOVER};}
function rsfClassTraces(classes){const s=D.sources.rsf_odom,cls=D.rsf.cls,[i,j]=range(s),th=D.meta.jump_thresh,out=[];
for(const c of classes){const x=[],y=[],t=[];let on=false;
for(let k=i;k<j;k++){const now=cls[k]===c;
if(now){const jump=k>i&&Math.hypot(s.x[k]-s.x[k-1],s.y[k]-s.y[k-1])>th;
if(!on&&k>i&&!jump){x.push(s.x[k-1]);y.push(s.y[k-1]);t.push(s.t[k-1]);}else if(on&&jump){x.push(null);y.push(null);t.push(null);}
x.push(s.x[k]);y.push(s.y[k]);t.push(s.t[k]);}
else if(on){x.push(null);y.push(null);t.push(null);}
on=now;}
out.push({type:'scattergl',mode:'lines',name:D.rsf.cls_labels[c],x,y,customdata:t,line:{color:D.rsf.cls_colors[c],width:3},connectgaps:false,hovertemplate:HOVER});}
return out;}
function jumpTrace(){const x=[],y=[],t=[];for(const j of D.rsf.jumps){if(j.t<S.t0||j.t>S.t1)continue;x.push(j.x0,j.x1,null);y.push(j.y0,j.y1,null);t.push(j.t,j.t,null);}
return {type:'scattergl',mode:'lines',name:'位置ジャンプ',x,y,customdata:t,line:{color:C.muted,width:1.5},connectgaps:false,hovertemplate:'ジャンプ @ %{customdata:.1f}s<extra></extra>'};}
function transitionTrace(){const f=D.rsf.transitions.filter(e=>e.t>=S.t0&&e.t<=S.t1);
return {type:'scattergl',mode:'markers',name:'切替点',x:f.map(e=>e.x),y:f.map(e=>e.y),text:f.map(e=>`${e.t.toFixed(1)}s ${e.detail}`),marker:{color:C.ink,size:8},hovertemplate:'%{text}<extra></extra>'};}
function endpointTrace(src){const p=pts(src);if(!p.x.length)return null;const n=p.x.length-1;
return {type:'scatter',mode:'markers+text',name:'始点 / 終点',x:[p.x[0],p.x[n]],y:[p.y[0],p.y[n]],text:[`START ${p.t[0].toFixed(0)}s`,`END ${p.t[n].toFixed(0)}s`],textposition:'bottom right',marker:{color:C.ink,size:11,symbol:['circle','square']},hoverinfo:'skip'};}
function gnssTraces(byQual){const s=D.sources.gnss;if(!s)return [];const [i,j]=range(s);
if(!byQual){const x=[],y=[],t=[];for(let k=i;k<j;k++){const fix=s.qual[k]===4;x.push(fix?s.x[k]:null);y.push(fix?s.y[k]:null);t.push(s.t[k]);}
return [{type:'scattergl',mode:'lines',name:'生 GNSS (RTK Fix のみ)',x,y,customdata:t,line:{color:C.muted,width:1.5},connectgaps:false,hovertemplate:HOVER}];}
const groups=[[4,'RTK Fix (4)',C.blue],[5,'Float (5)',C.orange],[2,'DGPS (2)',C.red],[null,'その他',C.muted]];
return groups.map(([q,name,color])=>{const x=[],y=[],t=[];for(let k=i;k<j;k++){const hit=q===null?![4,5,2].includes(s.qual[k]):s.qual[k]===q;if(hit){x.push(s.x[k]);y.push(s.y[k]);t.push(s.t[k]);}}
return {type:'scattergl',mode:'markers',name,x,y,customdata:t,marker:{color,size:5},hovertemplate:HOVER};}).filter(tr=>tr.x.length);}
function xyLayout(title,h){return {height:h||640,xaxis:{title:'x [m]',gridcolor:C.grid,zeroline:false},yaxis:{title:'y [m]',gridcolor:C.grid,zeroline:false,scaleanchor:'x',scaleratio:1},plot_bgcolor:C.surface,paper_bgcolor:C.surface,font:{family:'system-ui, sans-serif',color:C.ink,size:12},legend:{orientation:'h',y:1.0,yanchor:'bottom',x:0},hovermode:'closest',margin:{l:60,r:20,t:90,b:50}};}
function tsFigure(rows,h){const n=rows.length,gap=0.035,hgt=(1-gap*(n-1))/n,data=[];
const layout={height:h||Math.max(360,130*n+90),plot_bgcolor:C.surface,paper_bgcolor:C.surface,font:{family:'system-ui, sans-serif',color:C.ink,size:12},margin:{l:130,r:20,t:30,b:50},hovermode:'x',showlegend:true,legend:{orientation:'h',y:-0.04},annotations:[],xaxis:{title:'t [s]',gridcolor:C.grid,zeroline:false,range:[S.t0,S.t1],domain:[0,1],anchor:n>1?`y${n}`:'y'}};
rows.forEach((row,r)=>{const ax=r?`y${r+1}`:'y',dom=[1-(r+1)*hgt-r*gap,1-r*hgt-r*gap];
layout[r?`yaxis${r+1}`:'yaxis']={domain:dom,gridcolor:C.grid,zeroline:false,tickfont:{size:10},...(row.axis||{})};
layout.annotations.push({text:row.title,x:0,xref:'paper',xanchor:'left',y:dom[1],yref:'paper',yanchor:'bottom',showarrow:false,font:{size:12,color:C.ink2}});
for(const tr of row.traces){data.push({type:tr.strip?'scatter':'scattergl',mode:'lines',name:tr.name,x:tr.t,y:tr.v,xaxis:'x',yaxis:ax,line:{color:tr.color||C.blue,width:tr.strip?14:1.5,dash:tr.dash||'solid'},connectgaps:false,showlegend:!!tr.legend,hovertemplate:(tr.hover||'%{y}')+'<extra>'+tr.name+'</extra>'});}});
return {data,layout};}
function stateColor(v){v=String(v);if(v.includes('careful'))return C.violet;if(v==='normal'||v==='5')return C.orange;if(v==='2')return C.red;if(v.startsWith('LIO'))return C.aqua;if(v==='good'||v==='4'||v.startsWith('GNSS'))return C.blue;return C.muted;}
function stripRow(title,runs,colorOf){const vals=[...new Set(runs.map(r=>String(r[2])))];
return {title,axis:{type:'category',categoryorder:'array',categoryarray:vals,tickfont:{size:10}},traces:vals.map(v=>{const t=[],y=[];
for(const [a,b,val] of runs){if(String(val)!==v||b<S.t0||a>S.t1)continue;t.push(Math.max(a,S.t0),Math.min(b,S.t1),null);y.push(v,v,null);}
return {name:v,t,v:y,color:colorOf(v),strip:true,hover:'%{x:.1f}s'};})};}
function tabOverview(){const tr=rsfClassTraces([0,1,2,3]);
for(const [k,name,color,dash] of [['lio_lidar','LIO 生 (LiDAR レート)',C.ink2,'solid'],['lio_imu','LIO 生 (IMU レート)',C.ink2,'dot'],['utm','UTM odom',C.muted,'dash']]){if(D.sources[k])tr.push(trackTrace(D.sources[k],name,color,dash,1.5));}
tr.push(...gnssTraces(false),jumpTrace(),transitionTrace());const ep=endpointTrace(D.sources.rsf_odom);if(ep)tr.push(ep);
return [{id:'ov_xy',title:'全ソース重ね描き（rsf_odom 座標系）',fig:{data:tr,layout:xyLayout(null,720)}}];}
function tabRsf(){const s=D.sources.rsf_odom,tr=[...rsfClassTraces([0,1,2,3]),jumpTrace(),transitionTrace()];const ep=endpointTrace(s);if(ep)tr.push(ep);
const r=range(s),col=k=>s[k].slice(r[0],r[1]),t=col('t');
const rows=[{title:'速度 [m/s]',traces:[{name:'speed',t,v:col('speed')}]},{title:'z [m]',traces:[{name:'z',t,v:col('z')}]},
{title:'位置共分散 σ [m]',axis:{type:'log'},traces:[{name:'σx',t,v:col('sx'),legend:true},{name:'σy',t,v:col('sy'),color:C.orange,legend:true}]},
{title:'yaw [deg]',traces:[{name:'yaw',t,v:col('yaw')}]},stripRow('rsf_odom_type',D.strips.odom_type,stateColor),stripRow('rsf_odom_state',D.strips.odom_state,stateColor)];
return [{id:'rsf_xy',title:'rsf_odom: RTK Fix / Float / LIO / be careful',fig:{data:tr,layout:xyLayout(null,640)}},{id:'rsf_ts',fig:tsFigure(rows)}];}
function tabLioRaw(){const L=D.sources.lio_lidar,I=D.sources.lio_imu;if(!L&&!I)return [{id:'lio_none',html:'<p>LIO トピックがありません</p>'}];
const tr=[];if(L)tr.push(trackTrace(L,'LIO 生 (LiDAR レート)',C.blue,'solid',2.5));if(I)tr.push(trackTrace(I,'LIO 生 (IMU レート)',C.orange,'dot',2));tr.push(...gnssTraces(false));const ep=endpointTrace(L||I);if(ep)tr.push(ep);
const rows=[],zs=[];for(const [src,name,color] of [[L,'LiDAR レート',C.blue],[I,'IMU レート',C.orange]]){if(!src)continue;const r=range(src);zs.push({name,t:src.t.slice(...r),v:src.z.slice(...r),color,legend:true});}
rows.push({title:'z [m]',traces:zs});
if(L){const r=range(L),t=L.t.slice(...r);rows.push({title:'LIO 共分散 σ [m]（LiDAR レート）',axis:{type:'log'},traces:[{name:'σx',t,v:L.sx.slice(...r),legend:true},{name:'σy',t,v:L.sy.slice(...r),color:C.orange,legend:true},{name:'σz',t,v:L.sz.slice(...r),color:C.aqua,legend:true}]});}
if(D.series.cloud){const c=sl(D.series.cloud.t,D.series.cloud.npts);rows.push({title:'点群 点数 / フレーム',traces:[{name:'points',t:c.t,v:c.v}]});}
for(const [k,name,color] of [['lio_lidar','LiDAR レート',C.blue],['lio_imu','IMU レート',C.orange]]){const m=D.metrics[k];if(!m)continue;const g=sl(m.t,m.gap),h=sl(m.hdg_t,m.hdg_err),sc=sl(m.hdg_t,m.scale);
rows.push({title:`${name}: GNSS(Fix) との乖離 [m]（Fix 初期区間で位置合わせ）`,traces:[{name,t:g.t,v:g.v,color}]});
rows.push({title:`${name}: 方位誤差 [deg] / 距離スケール比 ×100`,traces:[{name:'方位誤差',t:h.t,v:h.v,color,legend:true},{name:'スケール比×100',t:sc.t,v:sc.v.map(v=>v===null?null:v*100),color:C.muted,dash:'dot',legend:true}]});}
return [{id:'lio_xy',title:S.align?'LIO 生（GNSS 基準に位置合わせ）':'LIO 生（出力そのまま, rsf_odom 座標系）',fig:{data:tr,layout:xyLayout(null,640)}},{id:'lio_ts',fig:tsFigure(rows)}];}
function tabLioRsf(){const tr=rsfClassTraces([2,3]);if(D.sources.lio_lidar)tr.push(trackTrace(D.sources.lio_lidar,'LIO 生 (LiDAR レート)',C.ink2,'solid',1.5,true));tr.push(...gnssTraces(false),jumpTrace(),transitionTrace());
const rows=[],m=D.metrics.rsf_vs_lio;
if(m){const o=sl(m.t,m.offset),mode=m.lio_mode.slice(lower(m.t,S.t0),lower(m.t,S.t1));
rows.push({title:'|rsf_odom − 純 LIO| [m]',traces:[{name:'LIO モード',t:o.t,v:o.v.map((v,i)=>mode[i]?v:null),color:C.aqua,legend:true},{name:'GNSS モード',t:o.t,v:o.v.map((v,i)=>mode[i]?null:v),color:C.blue,legend:true}]});
const h=sl(m.hdg_t,m.hdg_diff),sc=sl(m.hdg_t,m.scale);
rows.push({title:'rsf_odom と純 LIO の進行方向差 [deg] / 距離スケール比 ×100',traces:[{name:'方向差',t:h.t,v:h.v,legend:true},{name:'スケール比×100',t:sc.t,v:sc.v.map(v=>v===null?null:v*100),color:C.muted,dash:'dot',legend:true}]});}
const g=D.metrics.rsf_odom;if(g){const gg=sl(g.t,g.gap),h=sl(g.hdg_t,g.hdg_err);rows.push({title:'rsf_odom と生 GNSS(Fix) の乖離 [m]',traces:[{name:'gap',t:gg.t,v:gg.v}]});rows.push({title:'rsf_odom と GNSS の進行方向差 [deg]',traces:[{name:'方位誤差',t:h.t,v:h.v}]});}
rows.push(stripRow('rsf_odom_type',D.strips.odom_type,stateColor),stripRow('rsf_odom_state',D.strips.odom_state,stateColor));
return [{id:'lr_xy',title:'RSF 処理 LIO（rsf_odom の LIO 区間）と純 LIO（出力そのまま）',fig:{data:tr,layout:xyLayout(null,640)}},{id:'lr_ts',fig:tsFigure(rows)}];}
function span(a){let lo=Infinity,hi=-Infinity;for(const v of a){if(v===null)continue;if(v<lo)lo=v;if(v>hi)hi=v;}return hi-lo;}
function yawPanel(id,src,name,color,rot,extra){const p=pts(src),[i,j]=range(src),yaw=src.yaw.slice(i,j),len=0.05*Math.max(span(p.x),span(p.y))||1,step=Math.ceil(p.x.length/60),x=[],y=[],cd=[],size=[],ang=[];
for(let k=0;k<p.x.length;k+=step){if(yaw[k]===null)continue;const w=yaw[k]+rot,a=w*Math.PI/180;x.push(p.x[k],p.x[k]+len*Math.cos(a),null);y.push(p.y[k],p.y[k]+len*Math.sin(a),null);cd.push([p.t[k],w],[p.t[k],w],[null,null]);size.push(0,9,0);ang.push(0,90-w,0);}
const tr=[trackTrace(src,name,C.muted,'solid',1.5),{type:'scatter',mode:'lines+markers',name:'yaw 軸',x,y,customdata:cd,line:{color,width:2},marker:{symbol:'arrow',size,angle:ang,color},connectgaps:false,hovertemplate:'t=%{customdata[0]:.1f}s yaw=%{customdata[1]:.1f}°<extra></extra>'},...extra];
return {id,title:`${name}: yaw 軸（矢印 = 各時刻の機体 x 軸の向き, 表示範囲で約 60 本に間引き）`,fig:{data:tr,layout:xyLayout(null,520)}};}
function tabYaw(){const rotOf=k=>S.align&&D.align[k]?D.align[k].rot_deg:0,out=[yawPanel('yaw_rsf',D.sources.rsf_odom,'rsf_odom',C.ink,0,[jumpTrace(),transitionTrace()])];
if(D.sources.lio_lidar)out.push(yawPanel('yaw_lio',D.sources.lio_lidar,S.align?'LIO 生 (LiDAR レート, GNSS 基準に位置合わせ)':'LIO 生 (LiDAR レート, 出力そのまま)',C.aqua,rotOf('lio_lidar'),[]));
const wrap=v=>((v+540)%360)-180,yaws=[],diffs=[];
for(const [k,name,color] of [['rsf_odom','rsf_odom',C.ink],['lio_lidar','LIO 生 (LiDAR レート)',C.aqua],['lio_imu','LIO 生 (IMU レート)',C.orange]]){const src=D.sources[k];if(!src)continue;const [i,j]=range(src),t=src.t.slice(i,j),yaw=src.yaw.slice(i,j),cr=src.course.slice(i,j),r=k==='rsf_odom'?0:rotOf(k);
yaws.push({name,t,v:yaw.map(v=>v===null?null:wrap(v+r)),color,legend:true});diffs.push({name,t,v:yaw.map((v,q)=>v===null||cr[q]===null?null:wrap(v-cr[q])),color});}
out.push({id:'yaw_ts',fig:tsFigure([{title:'yaw [deg]'+(S.align?'（LIO は位置合わせの回転を加算）':''),traces:yaws},{title:'yaw − 進行方向 [deg]（1 s 間の移動方向との差, 移動 0.3 m 未満は除外）',traces:diffs}])});return out;}
function tabSim(){const s=D.sim;if(!s)return [{id:'sim_none',html:'<p>GNSS (RTK Fix) と LIO (LiDAR レート) の両方が必要です</p>'}];
const [i,j]=range(s),seg=m=>({type:'scattergl',mode:'lines',name:m?'理想切替: Fix 途切れ中（純 LIO の相対移動を積算）':'理想切替: GNSS Fix 区間',x:s.x.slice(i,j).map((v,k)=>s.mode[i+k]===m?v:null),y:s.y.slice(i,j).map((v,k)=>s.mode[i+k]===m?v:null),customdata:s.t.slice(i,j),line:{color:m?C.aqua:C.blue,width:2.5},connectgaps:false,hovertemplate:HOVER});
const J=s.jumps.filter(e=>e.t>=S.t0&&e.t<=S.t1),x=[],y=[],txt=[];for(const e of J){const h=`復帰 ${e.t}s: 跳び ${e.jump_m} m（途切れ ${e.outage_s} s, 走行 ${e.travel_m} m）`;x.push(e.x0,e.x1,null);y.push(e.y0,e.y1,null);txt.push(h,h,'');}
const tr=[trackTrace(D.sources.rsf_odom,'rsf_odom（デバイス出力, 切替ジャンプあり）',C.orange,'solid',1.5,true),...gnssTraces(false),seg(0),seg(1),{type:'scatter',mode:'lines+markers',name:'復帰時の跳び',x,y,text:txt,line:{color:C.ink,width:1.5,dash:'dash'},marker:{color:C.ink,size:7},connectgaps:false,hovertemplate:'%{text}<extra></extra>'}];
const rows=[],g=D.metrics.rsf_odom;if(g){const gg=sl(g.t,g.gap);rows.push({title:'|rsf_odom − RTK Fix| [m]（デバイス出力）',traces:[{name:'rsf_odom',t:gg.t,v:gg.v,color:C.orange}]});}
const c=sl(s.corr_t,s.corr);rows.push({title:'理想切替: Fix 更新ごとの補正量 [m]（復帰時の値 = 跳び幅）',traces:[{name:'補正量',t:c.t,v:c.v}]},stripRow('Fix 途切れ（2 s 超）',s.jumps.map(e=>[e.t-e.outage_s,e.t,'途切れ']),()=>C.aqua));
const tbl=J.map(e=>`<tr class="ev" data-t="${e.t}"><td>${e.t}</td><td>${e.outage_s}</td><td>${e.travel_m}</td><td>${e.jump_m}</td><td>${(e.jump_m/Math.max(e.travel_m,1)*100).toFixed(1)}</td></tr>`).join('');
return [{id:'sim_xy',title:'切替ジャンプを除いた理想的な GNSS/LIO 切替（RTK Fix を採用、途切れ中は純 LIO の相対移動を積算、復帰で GNSS 座標へ）',fig:{data:tr,layout:xyLayout(null,640)}},{id:'sim_ts',fig:tsFigure(rows)},
{id:'sim_tbl',html:`<div class="tiles"><div class="tile"><div class="k">Fix 継続中の毎更新補正 中央値 / 95%</div><div class="v">${s.routine_median_m} / ${s.routine_p95_m} m</div></div><div class="tile"><div class="k">Fix 途切れ（2 s 超）</div><div class="v">${s.jumps.length} 回</div></div></div>
<h3>Fix 復帰時の跳び（行クリックで前後 ±30 s を表示）</h3><table><tr><th>復帰 t [s]</th><th>途切れ [s]</th><th>途切れ中の走行 [m]</th><th>跳び [m]</th><th>跳び / 走行 [%]</th></tr>${tbl||'<tr><td colspan="5">該当なし</td></tr>'}</table>`}];}
function tabGnss(){const tr=gnssTraces(true);if(D.sources.utm)tr.push(trackTrace(D.sources.utm,'UTM odom',C.muted,'dash',1.5,true));
const rows=[],g=D.series.gps;
if(g){rows.push(stripRow('GPGGA gps_qual',D.strips.gps_qual,stateColor));const sats=sl(g.t,g.sats),h=sl(g.t,g.hdop),age=sl(g.t,g.diff_age);
rows.push({title:'衛星数',traces:[{name:'sats',t:sats.t,v:sats.v}]});rows.push({title:'HDOP / 補正経過時間 diff_age [s]',traces:[{name:'HDOP',t:h.t,v:h.v,legend:true},{name:'diff_age',t:age.t,v:age.v,color:C.orange,legend:true}]});}
const n=D.sources.gnss;
if(n){const r=range(n),f=D.series.rsf_fix;const sig=[{name:'nav_sat_fix',t:n.t.slice(...r),v:n.sigma.slice(...r),legend:true}],alt=[{name:'nav_sat_fix',t:n.t.slice(...r),v:n.z.slice(...r),legend:true}];
if(f){const s2=sl(f.t,f.sigma),a2=sl(f.t,f.alt);sig.push({name:'rsf_fix (切替出力)',t:s2.t,v:s2.v,color:C.orange,legend:true});alt.push({name:'rsf_fix (切替出力)',t:a2.t,v:a2.v,color:C.orange,legend:true});}
rows.push({title:'位置 σ [m]（NavSatFix covariance）',axis:{type:'log'},traces:sig});rows.push({title:'高度 [m]',traces:alt});}
if(D.series.gprmc){const sp=sl(D.series.gprmc.t,D.series.gprmc.speed),trk=sl(D.series.gprmc.t,D.series.gprmc.track);rows.push({title:'GPRMC speed / track（生値）',traces:[{name:'speed',t:sp.t,v:sp.v,legend:true},{name:'track',t:trk.t,v:trk.v,color:C.orange,dash:'dot',legend:true}]});}
return [{id:'g_xy',title:'生 GNSS（品質別）と UTM odom（rsf_odom 座標系に合わせ込み）',fig:{data:tr,layout:xyLayout(null,600)}},{id:'g_ts',fig:tsFigure(rows)}];}
function tabStates(){const rows=[];
for(const [k,title] of [['odom_type','rsf_odom_type'],['odom_state','rsf_odom_state'],['fix_type','rsf_fix_type'],['fix_state','rsf_fix_state'],['gps_qual','GPGGA gps_qual'],['diag','diagnostics (name level message)']]){if(D.strips[k]&&D.strips[k].length)rows.push(stripRow(title,D.strips[k],stateColor));}
const d=D.series.diag;if(d){for(const k of Object.keys(d)){if(k==='t')continue;const s=sl(d.t,d[k]);rows.push({title:`diagnostics: ${k}`,traces:[{name:k,t:s.t,v:s.v}]});}}
const im=D.series.imu;if(im){const gz=sl(im.t,im.gyro_z),g=sl(im.t,im.gyro),a=sl(im.t,im.acc);rows.push({title:'IMU 角速度 [deg/s]',traces:[{name:'|ω|',t:g.t,v:g.v,legend:true},{name:'ωz',t:gz.t,v:gz.v,color:C.orange,legend:true}]});rows.push({title:'IMU 加速度ノルム [m/s²]',traces:[{name:'|a|',t:a.t,v:a.v}]});}
const gaps=Object.entries(D.meta.gaps).sort().map(([k,v])=>`<tr><td>${k}</td><td>${(D.meta.stored[k]||{}).rate_hz??''}</td><td>${v.max_s.toFixed(2)}</td><td>${v.over_1s.length}</td></tr>`).join('');
return [{id:'st_ts',fig:tsFigure(rows)},{id:'st_gaps',html:`<h3>トピック受信間隔（欠落検出）</h3><table><tr><th>topic</th><th>平均 Hz</th><th>最大間隔 [s]</th><th>1 s 超の間隔 回数</th></tr>${gaps}</table>`}];}
function tabEvents(){const s=D.summary;
const tiles=[['記録時間',`${s.duration_s} s`],['経路長（ジャンプ除く）',`${s.path_m} m`],['平均速度',`${s.mean_speed} m/s`],['位置ジャンプ',`${s.n_jumps} 回 / 計 ${s.jump_total_m} m`]];
if(s.lio_drift)tiles.push(['純 LIO ドリフト（終端）',`${s.lio_drift.gap_end_m.toFixed(1)} m / ${s.lio_drift.dist_end_m.toFixed(0)} m 走行 (${s.lio_drift.gap_per_dist_pct}%)`]);
const tileHtml=tiles.map(([k,v])=>`<div class="tile"><div class="k">${k}</div><div class="v">${v}</div></div>`).join('');
const cls=Object.values(s.classes).map(c=>`<tr><td><span class="sw" style="background:${c.color}"></span>${c.label}</td><td>${c.share}%</td><td>${c.time_s} s</td><td>${c.dist_m} m</td></tr>`).join('');
const ep=s.fix_back_episodes.map(e=>`<tr><td>${e.t_fix}</td><td>${e.t_back??'—'}</td><td>${e.wait_s!==null?e.wait_s+' s':`復帰せず（終了まで ${e.until_end_s} s）`}</td></tr>`).join('');
const al=Object.entries(D.align).map(([k,v])=>`<tr><td>${k}</td><td>${v.method||(v.window_s?`Fix ${v.window_s[0]}–${v.window_s[1]} s`:'')}</td><td>${v.n}</td><td>${v.rms_m}</td><td>${v.rot_deg}</td></tr>`).join('');
const ev=D.events.map(e=>`<tr class="ev" data-t="${e.t}"><td>${e.t.toFixed(1)}</td><td>${e.kind}</td><td>${e.detail}</td></tr>`).join('');
return [{id:'ev_html',html:`<div class="tiles">${tileHtml}</div>
<h3>rsf_odom クラス別</h3><table><tr><th>クラス</th><th>割合</th><th>時間</th><th>距離</th></tr>${cls}</table>
<h3>GNSS Fix 復帰 → odom が GNSS に戻るまで</h3><table><tr><th>Fix 復帰 [s]</th><th>GNSS へ復帰 [s]</th><th>待ち時間</th></tr>${ep||'<tr><td colspan="3">該当なし</td></tr>'}</table>
<h3>座標系の位置合わせ</h3><table><tr><th>対象</th><th>基準</th><th>n</th><th>残差 rms [m]</th><th>回転 [deg]</th></tr>${al}</table>
<h3>イベント（行クリックで前後 ±30 s を概要に表示）</h3><table><tr><th>t [s]</th><th>種別</th><th>内容</th></tr>${ev}</table>`}];}
const BUILD={overview:tabOverview,rsf:tabRsf,lio_raw:tabLioRaw,lio_rsf:tabLioRsf,yaw:tabYaw,sim:tabSim,gnss:tabGnss,states:tabStates,events:tabEvents};
function render(tab){const panel=document.getElementById('panel_'+tab);panel.innerHTML='';
for(const item of BUILD[tab]()){const div=document.createElement('div');div.id=item.id;if(item.html){div.className='htmlpanel';div.innerHTML=item.html;}
if(item.title){const h=document.createElement('div');h.className='ptitle';h.textContent=item.title;div.appendChild(h);}
const plot=document.createElement('div');div.appendChild(plot);panel.appendChild(div);if(item.fig)Plotly.newPlot(plot,item.fig.data,item.fig.layout,{responsive:true,displaylogo:false});}
panel.querySelectorAll('tr.ev').forEach(tr=>tr.onclick=()=>{const t=+tr.dataset.t;setRange(Math.max(0,t-30),Math.min(D.meta.duration_s,t+30));if(tab==='events')show('overview');});
S.dirty[tab]=false;}
function show(tab){S.tab=tab;history.replaceState(null,'','#'+tab);
document.querySelectorAll('#tabs button').forEach(b=>b.classList.toggle('on',b.dataset.tab===tab));
document.querySelectorAll('.panel').forEach(p=>p.style.display=p.id==='panel_'+tab?'':'none');
if(S.dirty[tab]!==false)render(tab);}
function invalidate(){for(const [k] of TABS)S.dirty[k]=true;render(S.tab);}
function setRange(a,b){S.t0=a;S.t1=b;document.getElementById('t0').value=a.toFixed(0);document.getElementById('t1').value=b.toFixed(0);invalidate();}
(function init(){const nav=document.getElementById('tabs'),main=document.getElementById('panels');
for(const [k,label] of TABS){const b=document.createElement('button');b.textContent=label;b.dataset.tab=k;b.onclick=()=>show(k);nav.appendChild(b);const p=document.createElement('div');p.id='panel_'+k;p.className='panel';p.style.display='none';main.appendChild(p);}
const start=new Date(D.meta.t0_ns/1e6);const roles=Object.entries(D.meta.roles).filter(([,v])=>v).map(([k,v])=>`${k}: ${v}`).join('  ');
document.getElementById('meta').textContent=`${D.meta.bag_dir}  |  ${D.meta.duration_s.toFixed(0)} s  |  開始 ${start.toLocaleString('ja-JP')}  |  ${roles}`;
const ai=D.align.lio_lidar;document.getElementById('alignInfo').textContent=ai?`（LiDAR レート LIO: Fix ${ai.window_s[0]}–${ai.window_s[1]} s で推定, 残差 rms ${ai.rms_m} m, 回転 ${ai.rot_deg}°）`:'';
const t0=document.getElementById('t0'),t1=document.getElementById('t1');t0.value=0;t1.value=D.meta.duration_s.toFixed(0);
const apply=()=>{const a=Math.max(0,+t0.value||0),b=Math.min(D.meta.duration_s,+t1.value||D.meta.duration_s);if(b>a)setRange(a,b);};
t0.onchange=apply;t1.onchange=apply;document.getElementById('reset').onclick=()=>setRange(0,D.meta.duration_s);
document.getElementById('align').onchange=e=>{S.align=e.target.checked;invalidate();};
show(TABS.some(([k])=>'#'+k===location.hash)?location.hash.slice(1):'overview');})();
</script>
</body>
</html>
'''


def write_html(report, out_path):
    payload = json.dumps(report, ensure_ascii=False, separators=(',', ':')).replace('</', '<\\/')
    html = TEMPLATE.replace('__TITLE__', report['meta']['name']).replace('__DATA__', payload)
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(html)
