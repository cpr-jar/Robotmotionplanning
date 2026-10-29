"""Fetch and pin reviewed upstream kinematics sources; no ROS/Xacro execution."""
import urllib.request,json,hashlib
from pathlib import Path
root=Path(__file__).resolve().parents[2]/'models/upstream'
specs={
 'ur':('UniversalRobots/Universal_Robots_ROS2_Description','rolling',['urdf/ur_macro.xacro','config/ur5e/default_kinematics.yaml','config/ur5e/joint_limits.yaml','LICENSE']),
 'abb':('ros-industrial/abb','noetic-devel',['abb_irb120_support/urdf/irb120_3_58_macro.xacro','abb_irb120_support/package.xml']),
 'fanuc':('ros-industrial/fanuc','noetic-devel',['fanuc_lrmate200id_support/urdf/lrmate200id_macro.xacro','fanuc_lrmate200id_support/package.xml'])}
manifest=[]
for key,(repo,branch,paths) in specs.items():
 try:
  info=json.load(urllib.request.urlopen('https://api.github.com/repos/'+repo+'/commits/'+branch,timeout=40));commit=info['sha']
 except Exception as e:print(key,'commit error',e,flush=True);continue
 for path in paths:
  url='https://raw.githubusercontent.com/'+repo+'/'+commit+'/'+path
  try:
   data=urllib.request.urlopen(url,timeout=40).read();target=root/key/path;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
   manifest.append(dict(key=key,repo=repo,commit=commit,path=path,url=url,sha256=hashlib.sha256(data).hexdigest()))
   print(key,path,len(data),flush=True)
  except Exception as e:print(key,path,'error',e,flush=True)
 for license in ['LICENSE','LICENSE.txt',key+'_irb120_support/LICENSE' if key=='abb' else 'fanuc_lrmate200id_support/LICENSE'] if key!='ur' else []:
  try:
   url='https://raw.githubusercontent.com/'+repo+'/'+commit+'/'+license;data=urllib.request.urlopen(url,timeout=20).read();target=root/key/license;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data);manifest.append(dict(key=key,repo=repo,commit=commit,path=license,url=url,sha256=hashlib.sha256(data).hexdigest()));print('license',key,license,flush=True);break
  except Exception:pass
(root/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
