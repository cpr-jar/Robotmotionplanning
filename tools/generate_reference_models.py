"""Recreate nominal geometry from pinned reviewed sources. Not a Xacro engine.
Only whitelisted scalar arithmetic is allowed, no eval, ROS or network use.
Requires PyYAML only for regeneration, not for the C++ build.
"""
from pathlib import Path
import ast,csv,hashlib,json,math,re,xml.etree.ElementTree as ET
import yaml
ROOT=Path(__file__).resolve().parents[1]; UP=ROOT/'models/upstream'
class Loader(yaml.SafeLoader):pass
Loader.add_constructor('!degrees',lambda loader,node: math.radians(float(loader.construct_scalar(node))))
def arithmetic(node,env):
    if isinstance(node,ast.Expression):return arithmetic(node.body,env)
    if isinstance(node,ast.Constant) and isinstance(node.value,(str,int,float)):return node.value
    if isinstance(node,ast.Name) and node.id in env:return env[node.id]
    if isinstance(node,ast.UnaryOp) and isinstance(node.op,(ast.USub,ast.UAdd)):
        value=arithmetic(node.operand,env);return -value if isinstance(node.op,ast.USub) else value
    if isinstance(node,ast.BinOp) and isinstance(node.op,(ast.Add,ast.Sub,ast.Mult,ast.Div)):
        a,b=arithmetic(node.left,env),arithmetic(node.right,env)
        if isinstance(node.op,ast.Add):return a+b
        if isinstance(node.op,ast.Sub):return a-b
        if isinstance(node.op,ast.Mult):return a*b
        return a/b
    if isinstance(node,ast.Call) and isinstance(node.func,ast.Name) and node.func.id=='radians' and len(node.args)==1 and not node.keywords:return math.radians(arithmetic(node.args[0],env))
    raise ValueError('Unsupported expression: '+ast.dump(node))
def expand(text,env):return re.sub(r'\$\{([^}]+)\}',lambda m:str(arithmetic(ast.parse(m.group(1),mode='eval'),env)),text)
def multiply(a,b):return [[sum(a[i][k]*b[k][j] for k in range(4))for j in range(4)]for i in range(4)]
def identity():return [[float(i==j)for j in range(4)]for i in range(4)]
def origin(xyz,rpy):
    r,p,y=rpy;sr,cr,sp,cp,sy,cy=math.sin(r),math.cos(r),math.sin(p),math.cos(p),math.sin(y),math.cos(y)
    return [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,xyz[0]],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,xyz[1]],[-sp,cp*sr,cp*cr,xyz[2]],[0.,0.,0.,1.]]
def fk(robot,q):
    incoming={j.find('child').get('link'):j for j in robot.findall('joint')};chain=[];tip='tool0'
    while tip!='base_link':joint=incoming[tip];chain.append(joint);tip=joint.find('parent').get('link')
    result=identity();index=0
    for joint in reversed(chain):
        raw=joint.find('origin');xyz=[float(x)for x in raw.get('xyz','0 0 0').split()] if raw is not None else [0]*3;rpy=[float(x)for x in raw.get('rpy','0 0 0').split()] if raw is not None else [0]*3
        result=multiply(result,origin(xyz,rpy))
        if joint.get('type')!='fixed':
            axis=[float(x)for x in joint.find('axis').get('xyz').split()];n=math.sqrt(sum(x*x for x in axis));x,y,z=[a/n for a in axis];s,c=math.sin(q[index]),math.cos(q[index]);v=1-c;index+=1
            result=multiply(result,[[x*x*v+c,x*y*v-z*s,x*z*v+y*s,0],[y*x*v+z*s,y*y*v+c,y*z*v-x*s,0],[z*x*v-y*s,z*y*v+x*s,z*z*v+c,0],[0.,0.,0.,1.]])
    return result
def quaternion(m):
    trace=sum(m[i][i]for i in range(3))
    if trace>0:
        scale=math.sqrt(trace+1)*2;return [scale/4,(m[2][1]-m[1][2])/scale,(m[0][2]-m[2][0])/scale,(m[1][0]-m[0][1])/scale]
    i=max(range(3),key=lambda i:m[i][i]);j=(i+1)%3;k=(i+2)%3;scale=math.sqrt(1+m[i][i]-m[j][j]-m[k][k])*2;q=[0.]*4;q[i+1]=scale/4;q[0]=(m[k][j]-m[j][k])/scale;q[j+1]=(m[i][j]+m[j][i])/scale;q[k+1]=(m[i][k]+m[k][i])/scale;return q

def generate():
    manifest=json.loads((UP/'manifest.json').read_text())
    for item in manifest:assert hashlib.sha256((UP/item['key']/item['path']).read_bytes()).hexdigest()==item['sha256']
    definitions=[('ur5e','ur','urdf/ur_macro.xacro'),('abb_irb120','abb','abb_irb120_support/urdf/irb120_3_58_macro.xacro'),('fanuc_lrmate200id','fanuc','fanuc_lrmate200id_support/urdf/lrmate200id_macro.xacro')]
    for name,key,source in definitions:
        env={'prefix':'','tf_prefix':'','pi':math.pi,'wrist_3_joint_type':'revolute'}
        if key=='ur':
            config=yaml.load((UP/key/'config/ur5e/default_kinematics.yaml').read_text(),Loader=Loader)['kinematics']
            for link,fields in config.items():
                if isinstance(fields,dict):
                    for field,value in fields.items():env[link+'_'+field]=value
            caps=yaml.load((UP/key/'config/ur5e/joint_limits.yaml').read_text(),Loader=Loader)['joint_limits']
            for joint,fields in caps.items():
                stem=joint if joint=='elbow_joint' else joint.removesuffix('_joint')
                for outkey,inkey in [('lower','min_position'),('upper','max_position'),('velocity','max_velocity'),('effort','max_effort')]:env[stem+'_'+outkey+'_limit']=fields[inkey]
        macro=next(e for e in ET.parse(UP/key/source).getroot()if e.tag.rsplit('}',1)[-1]=='macro')
        robot=ET.Element('robot',name=name)
        for link in macro.findall('link'):ET.SubElement(robot,'link',name=expand(link.get('name'),env))
        for raw in macro.findall('joint'):
            jointname=expand(raw.get('name'),env)
            if jointname=='base_joint':continue
            joint=ET.SubElement(robot,'joint',name=jointname,type=expand(raw.get('type'),env))
            for tag in ['origin','parent','child','axis','limit']:
                field=raw.find(tag)
                if field is None and tag=='limit' and key=='ur':field=raw.find('.//limit')
                if field is not None:ET.SubElement(joint,tag,{k:expand(v,env)for k,v in field.attrib.items()})
        directory=ROOT/'models'/name;directory.mkdir(parents=True,exist_ok=True);ET.indent(robot,space='  ')
        (directory/'robot.urdf').write_text('<?xml version="1.0"?>\n<!-- Modified kinematics-only reconstruction, 2026-10-29. Original authors and licenses: ../upstream/'+key+'. No meshes or inertial/drive ratings. -->\n'+ET.tostring(robot,encoding='unicode')+'\n')
        joints={j.get('name'):{'max_acceleration':1.,'max_jerk':5.,'max_velocity':.5}for j in robot.findall('joint')if j.get('type')!='fixed'}
        profile={'schema_version':1,'units':'SI','robot_name':name,'base_link':'base_link','tip_link':'tool0','joints':joints,'tcp_limits':{'max_linear_velocity':.25,'max_linear_acceleration':.5,'max_linear_jerk':2.,'max_angular_velocity':.5,'max_angular_acceleration':1.,'max_angular_jerk':5.}}
        (directory/'profile.yaml').write_text('# PROTOTYPE CAPS, NOT manufacturer acceleration/jerk ratings.\n'+yaml.safe_dump(profile,sort_keys=False))
        with (directory/'reference_poses.csv').open('w',newline='') as f:
            writer=csv.writer(f);writer.writerow(['case']+['q'+str(i)for i in range(6)]+['T'+str(i)+str(j)for i in range(4)for j in range(4)])
            for label,q in [('zero',[0.]*6),('regular',[.2,-.6,.7,.3,.8,-.2]),('alternate',[.4,-.4,.6,-.7,.9,.5])]:writer.writerow([label]+q+[v for row in fk(robot,q)for v in row])
        first=fk(robot,[.2,-.6,.7,.3,.8,-.2]);(ROOT/'programs').mkdir(exist_ok=True)
        program={'schema_version':1,'units':'SI','frame':'world','robot_profile':'../models/'+name+'/profile.yaml','cycle_s':.01,'speed':.75,'start':'home',
          'points':{'home':{'joints':[.2,-.6,.7,.3,.8,-.2]},'approach':{'joints':[.21,-.6,.7,.3,.8,-.2]},'work':{'position':[first[0][3]+.002,first[1][3]+.001,first[2][3]],'orientation':quaternion(first)}},
          'subprograms':{'process':[{'type':'LIN','target':'work','speed':.5},{'type':'CIRC','center_offset':[-.002,0,0],'normal':[0,0,1],'sweep_rad':.5,'speed':.5},{'type':'WAIT','seconds':.05}]},
          'steps':[{'type':'PTP','target':'approach'},{'type':'CALL','program':'process'},{'type':'PTP','target':'home'}]}
        (ROOT/'programs'/(name+'_demo.yaml')).write_text('# Reconstructed nominal SI program, quaternion [w,x,y,z].\n'+yaml.safe_dump(program,sort_keys=False))
        print(name,len(joints),'axes, zero FK',[fk(robot,[0]*6)[i][3]for i in range(3)])
if __name__=='__main__':generate()
