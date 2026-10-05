"""Preserve v7 utensil outlines; repair fork, reinforce spoon and key the faces."""
from pathlib import Path
from zipfile import ZipFile
import xml.etree.ElementTree as ET
import numpy as np
import manifold3d as mf
import trimesh
from repair_parts import repair_fork,as_manifold,as_trimesh

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'source/pribor-cutlery-v7.3mf'
NS={'m':'http://schemas.microsoft.com/3dmanufacturing/core/2015/02'}
FILES={'spoon':2,'fork':5,'knife':4}
CENTRES={'spoon':(.30994069,-27.53664167),'fork':(-.00000117,-27.57244787),'knife':(.00147636,-27.56816631)}
THICKNESS={'fork':4.,'knife':4.,'spoon':5.8,'washer':2.4}
R=14.75
LUG_HEIGHT=1.2
ROOT_FILLET=.25
SIDE_CLEARANCE=.25
POCKET_DEPTH=1.4


def load(name):
 with ZipFile(SOURCE) as z:r=ET.fromstring(z.read(f'3D/Objects/object_{FILES[name]}.model'))
 v=np.array([[float(x.get(k)) for k in ('x','y','z')] for x in r.findall('.//m:vertex',NS)])
 f=np.array([[int(x.get(k)) for k in ('v1','v2','v3')] for x in r.findall('.//m:triangle',NS)])
 return trimesh.Trimesh(v,f,process=False)


def profile(offset=0):
 points=[]
 radius=.2+offset
 for cx,cy,start in [(1.1,1.8,0),(-1.1,1.8,90),(-1.1,-1.8,180),(1.1,-1.8,270)]:
  for angle in np.linspace(start,start+90,9):
   t=np.radians(angle);points.append((cx+radius*np.cos(t),cy+radius*np.sin(t)))
 return np.array(points)


def loft(levels):
 arrays=[np.c_[profile(off),np.full(36,z)] for z,off in levels]
 vertices=np.vstack(arrays);faces=[]
 for k in range(len(levels)-1):
  for j in range(36):
   n=(j+1)%36;a=k*36+j;b=k*36+n;c=(k+1)*36+n;d=(k+1)*36+j
   faces.extend([(a,b,c),(a,c,d)])
 bottom=len(vertices);top=bottom+1
 vertices=np.vstack([vertices,[0,0,levels[0][0]],[0,0,levels[-1][0]]])
 for j in range(36):
  n=(j+1)%36;faces.append((bottom,n,j));faces.append((top,(len(levels)-1)*36+j,(len(levels)-1)*36+n))
 solid=mf.Manifold(mf.Mesh64(vertices,np.array(faces,dtype=np.uint64)))
 if solid.status()!=mf.Error.NoError:raise ValueError(solid.status())
 return solid


def lug(x,top):
 heights=np.linspace(0,ROOT_FILLET,13)
 levels=[(-.8,ROOT_FILLET)]
 for z in heights:
  off=ROOT_FILLET-np.sqrt(max(0,ROOT_FILLET**2-(ROOT_FILLET-z)**2))
  levels.append((float(z),float(off)))
 # .15mm upper lead-in retains a rounded corner.
 levels.extend([(1.,0),(1.2,-.15)])
 return loft(levels).translate((x,0,top))


def pocket(x):
 return loft([(-.02,ROOT_FILLET+SIDE_CLEARANCE),(0,ROOT_FILLET+SIDE_CLEARANCE),(.4,SIDE_CLEARANCE),(POCKET_DEPTH,SIDE_CLEARANCE)]).translate((x,0,0))


def mark(solid,top,label):
 # A shallow direction arrow and identifying strokes; no raised feature between stacked plates.
 arrow=mf.CrossSection([[(0,25.8),(-1.8,23.8),(-.55,23.8),(-.55,21.2),(.55,21.2),(.55,23.8),(1.8,23.8)]])
 solid=solid-arrow.extrude(.30).translate((0,0,top-.25))
 # One/two/three rounded bars identify fork/knife/spoon without tiny lettering.
 count={'fork':1,'knife':2,'spoon':3}[label]
 for i in range(count):
  bar=mf.CrossSection.square((2.0,.4),center=True).offset(.2,circular_segments=24).extrude(.30)
  solid=solid-bar.translate(((i-(count-1)/2)*3.0,19.4,top-.25))
 return solid


def keyed(mesh,name,top):
 solid=as_manifold(mesh)
 solid=solid+lug(R,top)+lug(-R,top)
 solid=solid-pocket(R)-pocket(-R)
 return as_trimesh(mark(solid,top,name))


def washer():
 solid=mf.Manifold.cylinder(2.4,17.5,circular_segments=256)-mf.Manifold.cylinder(4.4,12,circular_segments=256).translate((0,0,-1))
 solid=solid-pocket(R)-pocket(-R)
 # Three shallow witness grooves on the upper outer edge identify pocketed face-down orientation.
 for ang in (-35,0,35):
  groove=mf.Manifold.cube((.8,2,.3),center=True).translate((0,16.4,2.3)).rotate((0,0,ang))
  solid=solid-groove
 return as_trimesh(solid)


def build_utensils():
 raw={n:load(n) for n in FILES};raw['fork'],evidence=repair_fork(raw['fork'])
 v=raw['spoon'].vertices.copy();v[:,2]-=.8*np.clip((-1.5-v[:,2]),0,1)
 raw['spoon']=trimesh.Trimesh(v,raw['spoon'].faces,process=False)
 parts={}
 for n in ['fork','knife','spoon']:
  m=raw[n];cx,cy=CENTRES[n];m.apply_translation((-cx,-cy,-m.bounds[0,2]))
  parts[n]=keyed(m,n,THICKNESS[n])
 parts['washer']=washer()
 return parts,evidence


if __name__=='__main__':
 from repair_parts import export_checked
 out=ROOT/'intermediate';out.mkdir(exist_ok=True)
 parts,evidence=build_utensils()
 for n,m in parts.items():
  export_checked(m,out/f'{n}-v9.stl')
  print(n,len(m.faces),round(m.volume,3),m.extents.tolist())
