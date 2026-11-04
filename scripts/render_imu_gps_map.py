#!/usr/bin/env python3
"""Render the extracted GPS reference map, completely offline."""
import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np


def render(path):
    data = json.loads(path.read_text())
    origin = data['metadata']['origin_lonlat']
    scale = np.array([math.pi/180*6378137*math.cos(math.radians(origin[1])),math.pi/180*6378137])
    font = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'axes.unicode_minus':False,'font.family':'Noto Sans CJK JP',
                         'font.size':10,'savefig.facecolor':'#f5f8fb'})
    fig = plt.figure(figsize=(16,10),facecolor='#f5f8fb')
    layout = fig.add_gridspec(2,2,width_ratios=[1.05,1],hspace=.30,wspace=.20)
    axes = [fig.add_subplot(layout[:,0]),fig.add_subplot(layout[0,1]),fig.add_subplot(layout[1,1])]
    selected = [lambda g:True,lambda g:g.startswith('taxiway'),lambda g:not g.startswith('taxiway')]
    titles = ['采集区域总览','滑行道：两次采集及补充','机位：505 / 506 / 507 及合并采集']
    for ax,choose,title in zip(axes,selected,titles):
        groups = set()
        ax.set_facecolor('white')
        for feature in data['features']:
            props = feature['properties']; group = props['group']
            if not choose(group):
                continue
            xy = (np.array(feature['geometry']['coordinates'])-origin)*scale
            label = props['group_label'] if group not in groups else None
            groups.add(group)
            ax.plot(xy[:,0],xy[:,1],color=props['color'],lw=1.45,alpha=.85,label=label)
            ax.scatter(*xy[0],color=props['color'],s=13,zorder=3)
            ax.scatter(*xy[-1],color=props['color'],marker='x',s=20,zorder=3)
        ax.set_aspect('equal',adjustable='datalim')
        ax.set_title(title,fontproperties=font,fontsize=13,pad=12,loc='left',color='#182b44')
        ax.grid(color='#d9e2eb',alpha=.75,lw=.6)
        ax.set_xlabel('东向 / m',fontproperties=font);ax.set_ylabel('北向 / m',fontproperties=font)
        ax.ticklabel_format(useOffset=False,style='plain')
        ax.legend(prop=font,loc='best',framealpha=.90)
        ax.text(.97,.96,'N ↑',transform=ax.transAxes,ha='right',va='top',fontsize=13,color='#182b44')
        for spine in ax.spines.values():spine.set_color('#cbd5df')
    fig.suptitle('GPS 实测轨迹地图',x=.07,ha='left',y=.985,fontproperties=font,fontsize=24,color='#142b45')
    fig.text(.07,.943,f"WGS84 · {data['metadata']['unique_nonempty_files']} 段采集 · {data['metadata']['unique_samples']:,} 条原始 GPS 记录 · 仅参考图层",
             fontproperties=font,fontsize=12,color='#52667b')
    fig.subplots_adjust(left=.07,right=.97,top=.89,bottom=.12)
    fig.text(.07,.055,'圆点：分段起点   ×：分段终点。原始分段保持独立；未闭合轨迹、未推断通行边界或连接道路。',
             fontproperties=font,fontsize=10,color='#52667b')
    fig.text(.07,.027,f'局部米制显示原点：经度 {origin[0]:9.8f}°，纬度 {origin[1]:8.8f}°。完整坐标保存在同目录 CSV。',
             fontproperties=font,fontsize=10,color='#52667b')
    fig.savefig(path.parent/'地图预览.png',dpi=160)
    fig.savefig(path.parent/'地图预览.svg')
    plt.close(fig)
    payload = json.dumps(data,ensure_ascii=False).replace('</','<\/')
    template = '''<!doctype html>
<html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>IMU GPS 实测轨迹地图</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#f2f6fa;color:#18314d;font-family:system-ui,"Noto Sans CJK SC",sans-serif}
header{padding:22px 30px 16px;background:white;border-bottom:1px solid #d9e3ed}h1{font-size:25px;margin:0 0 6px}p{margin:5px 0;color:#566b81;font-size:14px}
main{display:grid;grid-template-columns:270px 1fr;gap:16px;padding:18px;height:calc(100vh - 132px);min-height:520px}
aside,.map{background:white;border:1px solid #d8e2ec;border-radius:12px;overflow:hidden}aside{padding:20px;overflow-y:auto}h2{font-size:16px;margin:0 0 14px}
label{display:flex;gap:9px;align-items:center;margin:15px 0;font-size:14px}button{border:1px solid #c9d7e5;background:#f5f8fc;color:#18314d;padding:8px 11px;border-radius:7px;cursor:pointer;margin:0 4px 8px 0}
.badge{font-size:12px;color:#147658;background:#e8f5ef;padding:4px 8px;border-radius:5px;display:inline-block;margin:12px 0}.note{font-size:13px;line-height:1.7;color:#5a7085}.map{position:relative}svg{width:100%;height:100%;touch-action:none;cursor:grab;background:#fafcfe}svg:active{cursor:grabbing}
.readout{position:absolute;bottom:12px;left:14px;right:14px;background:#ffffffed;border:1px solid #d9e3ed;padding:9px 12px;border-radius:6px;font-size:13px;pointer-events:none}
.north{position:absolute;top:16px;right:20px;font-size:20px;pointer-events:none}.swatch{width:22px;height:3px;display:inline-block}#detail{margin-top:14px;font-size:13px;line-height:1.7;overflow-wrap:anywhere}
@media(max-width:800px){main{grid-template-columns:1fr;height:auto}aside{padding:15px}.map{height:65vh}label{display:inline-flex;margin-right:12px}}
</style>
<header><h1>GPS 实测轨迹地图</h1><p id="subtitle"></p></header>
<main><aside><h2>采集图层</h2><div id="layers"></div><button id="fit">查看全部</button><button id="taxiway">滑行道</button><button id="stands">机位</button><br>
<span class="badge">WGS84 · 离线地图</span><p class="note">滚轮缩放，拖动平移；指向线条可查看来源。下方显示鼠标对应的经纬度。圆点为起点，× 为终点。</p>
<p class="note">这是 GPS 实测参考轨迹。没有将蛇形采集线闭合为通行区域，也没有改变导航禁区。</p><div id="detail">选择图层查看采集信息。</div>
</aside><div class="map"><svg id="map" xmlns="http://www.w3.org/2000/svg"></svg><div class="north">N ↑</div><div class="readout" id="readout">拖动或滚轮缩放地图</div></div></main>
<script>const data=__PAYLOAD__;
const svg=document.getElementById('map'),NS='http://www.w3.org/2000/svg',origin=data.metadata.origin_lonlat;
const sx=Math.PI/180*6378137*Math.cos(origin[1]*Math.PI/180),sy=Math.PI/180*6378137;
const groups=new Map(),lines=[];let view=[0,0,1,1],drag=null;
const local=p=>[(p[0]-origin[0])*sx,-(p[1]-origin[1])*sy];
function element(tag,attrs){const n=document.createElementNS(NS,tag);Object.entries(attrs).forEach(([k,v])=>n.setAttribute(k,v));return n}
document.getElementById('subtitle').textContent=`${data.metadata.unique_nonempty_files} 段有效采集 · ${data.metadata.unique_samples.toLocaleString()} 条完整 GPS 记录 · 含滑行道、505 / 506 / 507 机位`;
for(const f of data.features){const p=f.properties,coords=f.geometry.coordinates.map(local),g=element('g',{});svg.appendChild(g);
const line=element('polyline',{points:coords.map(v=>v.join(',')).join(' '),fill:'none',stroke:p.color,'stroke-width':2,'vector-effect':'non-scaling-stroke'});g.appendChild(line);
const title=element('title',{});title.textContent=p.name;line.appendChild(title);
const first=coords[0],last=coords.at(-1),dot=element('circle',{cx:first[0],cy:first[1],r:.7,fill:p.color});g.appendChild(dot);
const cross=element('path',{d:`M${last[0]-1},${last[1]-1}l2,2m-2,0l2,-2`,stroke:p.color,'stroke-width':1.4,'vector-effect':'non-scaling-stroke'});g.appendChild(cross);
line.onpointerenter=()=>{document.getElementById('detail').textContent=`${p.name}；${p.records.toLocaleString()} 条记录；GPS_Status 原值分布：${JSON.stringify(p.gps_status_counts)}。`};
lines.push({group:p.group,coords,node:g});if(!groups.has(p.group))groups.set(p.group,{label:p.group_label,color:p.color,visible:true});}
for(const [key,g] of groups){const label=document.createElement('label'),box=document.createElement('input'),swatch=document.createElement('span');box.type='checkbox';box.checked=true;g.box=box;swatch.className='swatch';swatch.style.background=g.color;label.append(box,swatch,document.createTextNode(g.label));document.getElementById('layers').appendChild(label);box.onchange=()=>{g.visible=box.checked;draw()};}
function draw(){svg.setAttribute('viewBox',view.join(' '));lines.forEach(l=>l.node.style.display=groups.get(l.group).visible?'':'none');}
function fit(filter=()=>true){for(const [key,g] of groups){g.visible=filter(key);g.box.checked=g.visible}const pts=lines.filter(l=>groups.get(l.group).visible).flatMap(l=>l.coords),xs=pts.map(p=>p[0]),ys=pts.map(p=>p[1]);let x=Math.min(...xs),y=Math.min(...ys),w=Math.max(...xs)-x,h=Math.max(...ys)-y;const pad=Math.max(w,h)*.06+2;view=[x-pad,y-pad,w+2*pad,h+2*pad];draw()}
function point(e){const p=svg.createSVGPoint();p.x=e.clientX;p.y=e.clientY;const q=p.matrixTransform(svg.getScreenCTM().inverse());return[q.x,q.y]}
svg.onwheel=e=>{e.preventDefault();const p=point(e),k=e.deltaY>0?1.16:1/1.16;view=[p[0]+(view[0]-p[0])*k,p[1]+(view[1]-p[1])*k,view[2]*k,view[3]*k];draw()};
svg.onpointerdown=e=>{drag=point(e);svg.setPointerCapture(e.pointerId)};svg.onpointerup=()=>drag=null;svg.onpointercancel=()=>drag=null;
svg.onpointermove=e=>{const p=point(e);if(drag){view[0]+=drag[0]-p[0];view[1]+=drag[1]-p[1];draw()}document.getElementById('readout').textContent=`经度 ${(p[0]/sx+origin[0]).toFixed(8)}°　纬度 ${(-p[1]/sy+origin[1]).toFixed(8)}°　WGS84`};
document.getElementById('fit').onclick=()=>fit();document.getElementById('taxiway').onclick=()=>fit(g=>g.startsWith('taxiway'));document.getElementById('stands').onclick=()=>fit(g=>!g.startsWith('taxiway'));fit();
</script></html>'''
    (path.parent/'地图查看.html').write_text(template.replace('__PAYLOAD__',payload))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('geojson',type=Path)
    render(parser.parse_args().geojson)
