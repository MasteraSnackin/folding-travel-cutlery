"""Independent v9 integration audit using reloaded, exported STL geometry.

This checks model assembly and motion, not a physical print, strength or hygiene.
Geometry builders are deliberately not imported. The interface dimensions below
are the engineering assembly contract and exported input hashes are recorded.
"""
from __future__ import annotations
import hashlib
import itertools
import json
from pathlib import Path
import numpy as np
import trimesh
import manifold3d as mf

ROOT=Path(__file__).resolve().parent
NATIVE_INPUTS={**{n:ROOT/'intermediate'/f'{n}-v9.stl' for n in ['fork','knife','spoon','washer']},
        **{n:ROOT/'intermediate/pivot'/f'{n}-assembled.stl' for n in ['barrel','cap','keeper']},
        **{n:ROOT/'intermediate/cover'/f'{n}.stl' for n in ['cover','cover_key']}}
THICKNESS={'fork':4.,'knife':4.,'spoon':5.8,'washer':2.4}
TOL_VOLUME=1e-5
TOL_CONTACT_DEPTH=2e-5
ROUNDING=[]


def load(path,transform=None):
    mesh=trimesh.load_mesh(path,process=True)
    if transform is not None:mesh.apply_transform(transform)
    assert mesh.is_watertight and mesh.is_winding_consistent and mesh.volume>0,path
    solid=mf.Manifold(mf.Mesh64(np.asarray(mesh.vertices,dtype=np.float64),np.asarray(mesh.faces,dtype=np.uint64)))
    assert solid.status()==mf.Error.NoError and len(solid.decompose())==1,path
    return mesh,solid


def overlap(a,b,label=''):
    aa=np.asarray(a.bounding_box());bb=np.asarray(b.bounding_box())
    if np.any(aa[3:]<bb[:3]-1e-8) or np.any(bb[3:]<aa[:3]-1e-8):return 0.
    hit=a^b
    if hit.is_empty():return 0.
    vol=float(hit.volume())
    if vol<=TOL_VOLUME:return 0.
    box=np.asarray(hit.bounding_box());depth=float(box[5]-box[2])
    if depth<TOL_CONTACT_DEPTH:
        ROUNDING.append({'pair':label,'volume_mm3':vol,'z_depth_mm':depth});return 0.
    return vol


def pairs(solids):
    return {f'{a}/{b}':v for a,b in itertools.combinations(solids,2)
            if (v:=overlap(solids[a],solids[b],f'{a}/{b}'))>0}


def cap_pose(parts,rise):
    return parts['cap'].rotate((0,0,360*rise/2.5)).translate((0,0,rise))


def keeper_pose(parts,rise):
    return parts['keeper'].rotate((0,0,-360*rise/2.)).translate((0,0,rise))


def arrangement(parts,active=None,angle=180.,gap=0.,cover=True):
    out={};z=0.
    for i,n in enumerate(THICKNESS):
        out[n]=parts[n].rotate((0,0,angle if n==active else 0)).translate((0,0,z))
        z+=THICKNESS[n]
        if i<3:z+=gap
    out['barrel']=parts['barrel'];out['cap']=cap_pose(parts,3*gap);out['keeper']=parts['keeper']
    if cover:out['cover']=parts['cover'];out['cover_key']=parts['cover_key']
    return out


def driver_pose(parts,rise=0.,approach=0.,angle_error=0.):
    # Centre the end of the 8mm wide,2.2mm thick integral driver. Its slot
    # insertion is1.9mm, leaving0.1mm clearance to the keeper's slot floor.
    d=parts['cover_key'].translate((30.8,10.,3.)).rotate((0,-90,0)).translate((0,0,23.7))
    return d.rotate((0,0,-360*rise/2.+angle_error)).translate((0,0,rise+approach))


def main():
    inputs=NATIVE_INPUTS;inverse={};input_source='native intermediate STL exports'
    validation=ROOT/'validation.json'
    if validation.exists():
        metadata=json.loads(validation.read_text())
        if all((ROOT/'print'/f'{n}.stl').exists() for n in NATIVE_INPUTS):
            inputs={n:ROOT/'print'/f'{n}.stl' for n in NATIVE_INPUTS}
            inverse={n:np.linalg.inv(np.asarray(metadata['print_transforms'][n])) for n in inputs}
            for n,p in inputs.items():
                assert hashlib.sha256(p.read_bytes()).hexdigest()==metadata['parts'][n]['sha256'],f'Stale export metadata for {n}'
            input_source='final print STL exports, restored through recorded inverse print transforms'
    loaded={n:load(p,inverse.get(n)) for n,p in inputs.items()};parts={n:v[1] for n,v in loaded.items()}
    report={'scope':'Independent integration audit of exported meshes; no physical print, load, wear, fit or food-contact validation.',
            'units':'mm','input_source':input_source,'input_sha256':{n:hashlib.sha256(p.read_bytes()).hexdigest() for n,p in inputs.items()},
            'input_paths':{n:str(p.relative_to(ROOT)) for n,p in inputs.items()},
            'clamped_with_cover':{},'released_rotation':{},'keyed_rotation_blockers':{},
            'cap_above_stationary_stack':{},'full_cleaning_disassembly':{},'keeper_driver':{}}
    for active in [None,'fork','knife','spoon']:
        report['clamped_with_cover'][active or 'folded']=pairs(arrangement(parts,active))
    for active in ['fork','knife','spoon']:
        report['released_rotation'][active]={str(a):pairs(arrangement(parts,active,a,1.4,False)) for a in range(0,181,15)}
        report['keyed_rotation_blockers'][active]={str(a):pairs(arrangement(parts,active,a,0.,False)) for a in [-3,3]}
    released=arrangement(parts,gap=1.4,cover=False)
    ordered=list(THICKNESS)
    report['released_interpart_z_clearance_mm']={f'{a}/{b}':float(released[b].bounding_box()[2]-released[a].bounding_box()[5]) for a,b in zip(ordered,ordered[1:])}
    # Isolated cap/barrel/keeper sweeps are in pivot-validation.json; this
    # integration check focuses on the stationary keyed washer/utensil stack.
    stack={n:s for n,s in arrangement(parts,cover=False).items() if n in THICKNESS}
    for rise in [0.,.2,1.,2.5,4.2,4.6]:
        c=cap_pose(parts,rise)
        report['cap_above_stationary_stack'][str(rise)]={n:v for n,s in stack.items() if (v:=overlap(c,s,f'cap/{n}'))>0}
    # Full cleaning: keeper must escape through a closed cap; subsequently the
    # cap must unthread through the full length, not merely to the opening stop.
    report['full_cleaning_disassembly']['keeper_with_closed_cap']={}
    for rise in [0.,1.,3.,5.,8.,14.,20.,25.,30.,35.]:
        k=keeper_pose(parts,rise)
        report['full_cleaning_disassembly']['keeper_with_closed_cap'][str(rise)]={n:v for n in ['barrel','cap'] if (v:=overlap(k,parts[n],f'keeper/{n}'))>0}
    report['full_cleaning_disassembly']['cap_without_keeper']={}
    for rise in [0.,4.6,7.,10.,13.,16.,18.]:
        c=cap_pose(parts,rise)
        report['full_cleaning_disassembly']['cap_without_keeper'][str(rise)]={n:v for n,s in {'barrel':parts['barrel'],**stack}.items() if (v:=overlap(c,s,f'cap/{n}'))>0}
    report['full_cleaning_disassembly']['main_cap_bottom_at_18mm_rise']=float(cap_pose(parts,18.).bounding_box()[2])
    # Direct driver approach and keeper-unthreading poses include the complete
    # transverse key, not merely an ideal rectangle matching the blade.
    report['keeper_driver']['vertical_approach']={}
    for approach in [12.,6.,2.,.5,0.]:
        driver=driver_pose(parts,approach=approach)
        report['keeper_driver']['vertical_approach'][str(approach)]={n:v for n in ['barrel','cap','keeper'] if (v:=overlap(driver,parts[n],f'driver/{n}'))>0}
    report['keeper_driver']['clockwise_removal']={}
    for rise in [0.,.25,.5,1.,2.,3.,5.,8.]:
        driver=driver_pose(parts,rise);keeper=keeper_pose(parts,rise)
        obstacles={'barrel':parts['barrel'],'cap':parts['cap'],'keeper':keeper}
        report['keeper_driver']['clockwise_removal'][str(rise)]={n:v for n,s in obstacles.items() if (v:=overlap(driver,s,f'driver/{n}'))>0}
    report['keeper_driver']['intentional_10deg_slot_wall_interference_mm3']=overlap(driver_pose(parts,angle_error=10),parts['keeper'],'driver/slot wall')
    # Coaxial guided retention. Cap stop blocks lifting washer far enough to
    # clear the barrel; this is not an exhaustive6DOF escape or force analysis.
    max_clear_rise=4.6;nominal_stop_rise=4.61
    attempted_escape=parts['washer'].translate((0,0,21.05))
    bv=loaded['barrel'][0].vertices;guide_top=float(bv[:,2].max())
    guide_radius=float(np.hypot(bv[bv[:,2]>.1,0],bv[bv[:,2]>.1,1]).max())
    cv=loaded['cap'][0].vertices;cap_bearing=float(cv[np.hypot(cv[:,0],cv[:,1])>12.5,2].min())
    washer_top=float(released['washer'].bounding_box()[5]);washer_bottom=float(released['washer'].bounding_box()[2])
    measured_bore_radii={n:float(np.hypot(loaded[n][0].vertices[:,0],loaded[n][0].vertices[:,1]).min()) for n in THICKNESS}
    report['guide_and_capture']={
      'barrel_top_mm':guide_top,'barrel_guide_radius_mm':guide_radius,'normal_release_total_gap_mm':4.2,
      'normal_release_washer_top_mm':washer_top,'normal_release_top_guidance_margin_mm':guide_top-washer_top,
      'nominal_stop_rise_mm':nominal_stop_rise,'washer_top_at_nominal_stop_mm':cap_bearing+nominal_stop_rise,
      'washer_top_margin_at_nominal_stop_mm':guide_top-(cap_bearing+nominal_stop_rise),
      'minimum_vertex_bore_radii_mm':measured_bore_radii,
      'nominal_radial_bore_clearance_mm':{n:r-guide_radius for n,r in measured_bore_radii.items()},
      'washer_full_bore_guided_at_normal_release':washer_bottom>=0. and washer_top<guide_top,
      'cap_keeper_interference_beyond_stop_mm3':overlap(cap_pose(parts,4.7),parts['keeper'],'cap/keeper beyond stop'),
      'washer_escape_attempt_cap_interference_mm3':overlap(attempted_escape,cap_pose(parts,max_clear_rise),'washer/cap escape'),
      'scope':'Coaxial translation and prescribed opening configurations; no physical retention-force claim.'}
    report['contact_rounding']={'z_depth_tolerance_mm':TOL_CONTACT_DEPTH,'ignored_count':len(ROUNDING),
      'max_z_depth_mm':max([v['z_depth_mm'] for v in ROUNDING],default=0.)}
    no_collision_maps=[*report['clamped_with_cover'].values(),
        *(x for row in report['released_rotation'].values() for x in row.values()),
        *report['cap_above_stationary_stack'].values(),
        *report['full_cleaning_disassembly']['keeper_with_closed_cap'].values(),
        *report['full_cleaning_disassembly']['cap_without_keeper'].values(),
        *report['keeper_driver']['vertical_approach'].values(),
        *report['keeper_driver']['clockwise_removal'].values()]
    report['passes']=all(not x for x in no_collision_maps) and all(v>0 for v in report['released_interpart_z_clearance_mm'].values()) and all(x for row in report['keyed_rotation_blockers'].values() for x in row.values()) and report['keeper_driver']['intentional_10deg_slot_wall_interference_mm3']>0 and report['guide_and_capture']['cap_keeper_interference_beyond_stop_mm3']>0 and report['guide_and_capture']['washer_escape_attempt_cap_interference_mm3']>0 and report['guide_and_capture']['washer_full_bore_guided_at_normal_release'] and report['guide_and_capture']['washer_top_margin_at_nominal_stop_mm']>0 and all(v>0 for v in report['guide_and_capture']['nominal_radial_bore_clearance_mm'].values())
    (ROOT/'integration-audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)
    assert report['passes'],'v9 integration audit failed; inspect integration-audit.json'

if __name__=='__main__':main()
