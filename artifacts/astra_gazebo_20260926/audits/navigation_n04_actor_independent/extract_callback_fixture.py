from pathlib import Path
import hashlib,json
D=Path(__file__).resolve().parent;S=D.parents[1];P=S/'workspace/src/simulation/m2_gazebo/src/astra_event_actor_n04_plugin.cpp';text=P.read_text()
parts=[]
for signature in ('void receive(','void consumeEvent()','void publishStatus()'):
 start=text.index('  '+signature);brace=text.index('{',start);depth=1;i=brace+1
 while depth:
  if text[i]=='{':depth+=1
  if text[i]=='}':depth-=1
  i+=1
 parts.append(text[start:i])
(D/'actual_callback_bodies.inc').write_text('\n'.join(parts)+'\n')
(D/'extraction.json').write_text(json.dumps({'source':str(P),'source_sha256':hashlib.sha256(P.read_bytes()).hexdigest(),'method_signatures':['receive','consumeEvent','publishStatus'],'exact_bodies_sha256':hashlib.sha256((D/'actual_callback_bodies.inc').read_bytes()).hexdigest(),'purpose':'compile exact callback/consumption/status bodies against explicit fake clocks/transport; synthetic controlflow evidence, no ROS or Gazebo'},indent=2)+'\n')
