import html
import json
from pathlib import Path


def write_report(output, report, telemetry):
    output = Path(output)
    (output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    with (output/'telemetry.jsonl').open('w') as stream:
        for row in telemetry:
            stream.write(json.dumps(row, allow_nan=False)+'\n')
    cards = ''.join(f'<figure><img src="{html.escape(c["file"], quote=True)}"><figcaption>{html.escape(c["name"])} · {c["time"]:.1f}s</figcaption></figure>' for c in report['captures'])
    payload = json.dumps(telemetry, allow_nan=False).replace('<','\\u003c')
    events = html.escape(json.dumps(report['events'],indent=2))
    page = '''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rover inspection replay</title><style>
body{margin:0;background:#0c1420;color:#e2eaf4;font:16px system-ui}main{max-width:1080px;margin:auto;padding:36px 24px}h1{font-size:34px;margin:8px 0}.muted{color:#9bb0c8} .badge{color:#73e4bd} .grid{display:grid;grid-template-columns:1fr 1fr;gap:18px}canvas{background:#142236;width:100%;border-radius:12px}button{background:#73e4bd;color:#10241e;border:0;padding:10px 20px;border-radius:7px;cursor:pointer}input{width:75%;vertical-align:middle}figure{margin:0}img{width:100%;border-radius:10px}figcaption{padding:10px 0}pre{background:#142236;padding:18px;border-radius:12px;overflow:auto;max-height:300px}#status{min-height:48px;padding:14px 0;font-variant-numeric:tabular-nums}.captures{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}@media(max-width:650px){.grid,.captures{grid-template-columns:1fr}}
</style><main><div class="badge">SCRIPTED INSPECTION · __MODE__</div><h1>Approach. Inspect. Recover.</h1><p class="muted">__SUMMARY__</p><p><strong>Mission: __RESULT__</strong></p><div class="grid"><section><h3>Route replay</h3><canvas id="map" width="480" height="320"></canvas></section><section><h3>Inspection observations</h3><canvas id="camera" width="480" height="320"></canvas></section></div><div id="status"></div><button id="play">Play</button> <input aria-label="Replay time" id="scrub" type="range" min="0" value="0"><h2>Captured views</h2><div class="captures">__CARDS__</div><h2>Mission events</h2><pre>__EVENTS__</pre></main><script>
const rows=__DATA__, slider=document.getElementById('scrub'), status=document.getElementById('status');
const colors=['#46d7af','#ffb94b','#7da0ff'];let index=0, playing=false,last=0;slider.max=Math.max(0,rows.length-1);
function draw(){if(!rows.length)return;const r=rows[index];slider.value=index;status.textContent=`${r.time.toFixed(1)} s · ${r.state} · Target ${r.target} · ${r.captures} captured · Camera ${r.fresh?'fresh':'unavailable / stale'}`;
let m=document.getElementById('map').getContext('2d');m.clearRect(0,0,480,320);const xy=p=>[45+p[0]*145,160-p[1]*200];m.strokeStyle='#375571';m.lineWidth=2;m.beginPath();rows.slice(0,index+1).forEach((v,i)=>{const p=xy(v.pose);i?m.lineTo(...p):m.moveTo(...p)});m.stroke();let [x,y]=xy(r.pose);m.fillStyle='#73e4bd';m.beginPath();m.arc(x,y,9,0,Math.PI*2);m.fill();m.strokeStyle='#fff';m.beginPath();m.moveTo(x,y);m.lineTo(x+25*Math.cos(r.pose[2]),y-25*Math.sin(r.pose[2]));m.stroke();m.fillStyle='#9bb0c8';m.fillRect(405,50,8,220);m.font='14px system-ui';m.fillText('Panel',390,35);m.fillText('Illustrative XY trajectory',16,302);
let c=document.getElementById('camera').getContext('2d');c.fillStyle=r.fresh?'#142236':'#371922';c.fillRect(0,0,480,320);Object.entries(r.boxes).forEach(([id,b])=>{let [x,y,w,h]=b;x=(x+1)*240;y=(y+1)*160;w*=480;h*=320;c.strokeStyle=colors[Number(id)%3];c.lineWidth=3;c.strokeRect(x-w/2,y-h/2,w,h);c.fillStyle=c.strokeStyle;c.font='14px system-ui';c.fillText('Target '+id,x-w/2,y-h/2-8)});c.strokeStyle='#fff';c.beginPath();c.moveTo(228,160);c.lineTo(252,160);c.moveTo(240,148);c.lineTo(240,172);c.stroke();c.fillStyle='#9bb0c8';c.font='12px system-ui';c.fillText('Observation diagram — see captured images below',14,303);}
slider.oninput=()=>{index=Number(slider.value);draw()};document.getElementById('play').onclick=()=>{if(index===rows.length-1)index=0;playing=!playing;document.getElementById('play').textContent=playing?'Pause':'Play'};
function animate(t){if(playing&&t-last>=100){last=t;if(index<rows.length-1){index++;draw()}else{playing=false;document.getElementById('play').textContent='Play'}}requestAnimationFrame(animate)}draw();requestAnimationFrame(animate);
</script></html>'''
    page = page.replace('__MODE__',html.escape(report['backend'].upper())).replace('__SUMMARY__',
        'Synthetic geometry and camera images; this is a software demonstration, not a physics or hardware validation.' if report['backend']=='simulation' else 'Recorded hardware mission. Replay displays detected boxes; captures below contain camera images.')
    page = page.replace('__RESULT__',html.escape('COMPLETE' if report['success'] else 'FAILED: '+str(report['failure_reason'])))
    page = page.replace('__CARDS__',cards).replace('__EVENTS__',events).replace('__DATA__',payload)
    (output/'replay.html').write_text(page)
