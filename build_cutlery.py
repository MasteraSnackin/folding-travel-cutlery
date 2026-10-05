"""Assemble and export the v9 fully printed folding-cutlery prototype."""
from pathlib import Path
from zipfile import ZipFile,ZIP_DEFLATED
import xml.etree.ElementTree as ET
import hashlib,json,math
import numpy as np
import trimesh
import manifold3d as mf
from repair_parts import as_manifold,as_trimesh,export_checked
from utensils_v9 import build_utensils,THICKNESS,ROOT_FILLET,SIDE_CLEARANCE,R,LUG_HEIGHT
import pivot_v9 as pivot
import cover_v9 as cover
ROOT=Path(__file__).resolve().parent
CORE='http://schemas.microsoft.com/3dmanufacturing/core/2015/02'
PRINT_ORDER=['fork','knife','spoon','cover','barrel','cap','washer','keeper','cover_key']


def build_parts():
 parts,evidence=build_utensils()
 parts.update(pivot.build_pivot(attachment_ears=cover.attachment_ears(),attachment_cutter=cover.attachment_cutter()))
 parts['cover']=as_trimesh(cover.build_cover())
 parts['cover_key']=as_trimesh(cover.build_latch_key())
 return parts,evidence


def assembly(parts,active=None,angle=180.,release=0.,with_cover=False):
 out={};z=0.;gap=release/3
 for i,n in enumerate(['fork','knife','spoon','washer']):
  m=parts[n].copy()
  if active==n:m.apply_transform(trimesh.transformations.rotation_matrix(np.deg2rad(angle),(0,0,1)))
  m.apply_translation((0,0,z));out[n]=m
  z+=THICKNESS[n]
  if i<3:z+=gap
 out['barrel']=parts['barrel'].copy()
 out['cap']=pivot.cap_pose(parts['cap'],release)
 out['keeper']=parts['keeper'].copy()
 if with_cover:
  if release:raise ValueError('Remove cover before opening the pivot')
  out['cover']=parts['cover'].copy();out['cover_key']=parts['cover_key'].copy()
 return out


def print_parts(parts):
 result={};transforms={}
 for name in PRINT_ORDER:
  m=parts[name].copy();t=np.eye(4)
  if name in ['cap','keeper']:t=trimesh.transformations.rotation_matrix(math.pi,(1,0,0))
  if name=='cover':t=cover.cover_print_transform()
  if name=='cover_key':t=cover.latch_key_print_transform()
  m.apply_transform(t)
  shift=np.eye(4);shift[2,3]=-m.bounds[0,2];m.apply_transform(shift)
  transforms[name]=(shift@t).tolist();result[name]=m
 return result,transforms


def layout(meshes,bed=256,margin=10,gap=8):
 positions={};x=margin;y=margin;row=0
 for n,m in meshes.items():
  w,h=m.extents[:2]
  if x+w>bed-margin:x=margin;y+=row+gap;row=0
  if y+h>bed-margin:raise ValueError('Parts do not fit chosen bed')
  positions[n]=[float(x-m.bounds[0,0]),float(y-m.bounds[0,1]),0.]
  x+=w+gap;row=max(row,float(h))
 return positions


def write_3mf(meshes,path,title,description):
 ET.register_namespace('',CORE);q=lambda n:'{'+CORE+'}'+n
 root=ET.Element(q('model'),unit='millimeter')
 for name,value in [('Title',title),('Application','Cutlery v9 rebuild'),('Description',description)]:ET.SubElement(root,q('metadata'),name=name).text=value
 resources=ET.SubElement(root,q('resources'));build=ET.SubElement(root,q('build'));positions=layout(meshes)
 for i,(name,m) in enumerate(meshes.items(),1):
  ob=ET.SubElement(resources,q('object'),id=str(i),type='model',name=name)
  obj=ET.SubElement(ob,q('mesh'));vertices=ET.SubElement(obj,q('vertices'));faces=ET.SubElement(obj,q('triangles'))
  for v in m.vertices:ET.SubElement(vertices,q('vertex'),**dict(zip(('x','y','z'),(f'{x:.9f}' for x in v))))
  for f in m.faces:ET.SubElement(faces,q('triangle'),**dict(zip(('v1','v2','v3'),map(str,f))))
  x,y,z=positions[name];ET.SubElement(build,q('item'),objectid=str(i),transform=f'1 0 0 0 1 0 0 0 1 {x:.9f} {y:.9f} {z:.9f}')
 with ZipFile(path,'w',ZIP_DEFLATED) as z:
  z.writestr('[Content_Types].xml','<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
  z.writestr('_rels/.rels','<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
  z.writestr('3D/3dmodel.model',ET.tostring(root,encoding='utf-8',xml_declaration=True))
 return positions


def collisions(parts):
 solids={n:as_manifold(m) for n,m in parts.items()};names=list(solids);bad={};contacts=[]
 for i,a in enumerate(names):
  for b in names[i+1:]:
   hit=solids[a]^solids[b];v=float(hit.volume())
   if v>1e-5:
    bounds=hit.bounding_box();depth=bounds[5]-bounds[2]
    if depth<2e-5:contacts.append({'pair':a+'/'+b,'volume':v,'z_depth':depth})
    else:bad[a+'/'+b]=v
 return bad,contacts


def main():
 parts,evidence=build_parts();printed,transforms=print_parts(parts)
 out=ROOT/'print';out.mkdir(exist_ok=True)
 report={'units':'mm','version':'v9 prototype','source_sha256':hashlib.sha256((ROOT/'source/pribor-cutlery-v7.3mf').read_bytes()).hexdigest(),
 'parts':{},'print_transforms':transforms,'ring_thickness_mm':THICKNESS,'lug_centres_radius_mm':R,'lug_footprint_mm':[2.6,4.0],
 'root_fillet_mm':ROOT_FILLET,'tab_height_mm':LUG_HEIGHT,'pocket_side_clearance_mm':SIDE_CLEARANCE,
 'spoon_floor_mm':1.8,'stack_height_mm':16.2,'barrel_height_mm':21.,'barrel_diameter_mm':23.6,
 'release_mm':4.2,'release_turns':4.2/2.5,'tested_cap_limit_mm':4.6,'nominal_keeper_stop_mm':4.61,
 'closed_pivot_height_mm':31.6,'keeper_remove':'clockwise','cover_key_release':'push right0.8mm, quarter-turn, withdrawleft',
 'parts_count':len(parts),'fork_repair':evidence,'collisions_mm3':{},'contact_rounding':{}}
 for n,m in printed.items():report['parts'][n]=export_checked(m,out/f'{n}.stl')
 report['layout_mm']=write_3mf(printed,ROOT/'cutlery-v9-folding-prototype.3mf','Folding travel cutlery v9 — fully printed prototype','Nine separate parts. Fixed pivot barrel; captured cap and washer; removable left-hand keeper; mechanically latched cover. Unsliced geometry; physical performance and food contact unverified.')
 for active in [None,'fork','knife','spoon']:
  state=assembly(parts,active=active,with_cover=True)
  bad,contacts=collisions(state);name=active or 'folded';report['collisions_mm3'][name]=bad;report['contact_rounding'][name]=contacts
 folded=assembly(parts,with_cover=True);v=np.vstack([m.vertices for m in folded.values()]);report['folded_with_cover_bounds_mm']=[v.min(axis=0).tolist(),v.max(axis=0).tolist()]
 # Smaller mechanism test uses identical hardware/washer and cropped actual rings.
 test={};testdir=ROOT/'mechanism-test';testdir.mkdir(exist_ok=True)
 for n in ['fork','knife','spoon']:
  test['test_'+n]=as_trimesh(as_manifold(parts[n]).trim_by_plane((0,-1,0),-18.5))
 for n in ['barrel','cap','washer','keeper','cover_key']:test[n]=printed[n]
 for n,m in test.items():export_checked(m,testdir/f'{n}.stl')
 report['mechanism_test_layout_mm']=write_3mf(test,ROOT/'cutlery-v9-mechanism-test.3mf','Cutlery v9 — smaller mechanism test','Three cropped utensil rings and the exact full-size barrel,cap,washer,keeper,driver key. Unsliced prototype; print this to test fit before full utensils and cover.')
 report['passes']=all(not r for r in report['collisions_mm3'].values())
 (ROOT/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps({k:report[k] for k in ['parts_count','collisions_mm3','folded_with_cover_bounds_mm','passes']},indent=2))
 assert report['passes'],'Integrated geometry has interference; inspect report'

if __name__=='__main__':main()
