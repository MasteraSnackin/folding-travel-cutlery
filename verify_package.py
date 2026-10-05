"""Independent exported-package audit and optional clean-directory rebuild.

Run with the dependencies in requirements.txt:
    python verify_package.py
    python verify_package.py --rebuild

The package audit does not import any builder modules. It reads the actual
3MF/STL files, their transforms and preserved source. The optional rebuild
copies only the five build modules and the source directory into a temporary
directory, executes the builder there, and compares STL bytes plus all 3MF
payloads. ZIP timestamps are deliberately excluded from reproducibility.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
from itertools import combinations
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from zipfile import ZipFile

import manifold3d as mf
import numpy as np
import trimesh

ROOT=Path(__file__).resolve().parent
NS={'m':'http://schemas.microsoft.com/3dmanufacturing/core/2015/02'}
EXPECTED={
    'cutlery-v9-folding-prototype.3mf':('print',{
        'fork','knife','spoon','washer','barrel','cap','keeper','cover','cover_key'}),
    'cutlery-v9-mechanism-test.3mf':('mechanism-test',{
        'test_fork','test_knife','test_spoon','barrel','cap','washer','keeper','cover_key'}),
}
BUILD_MODULES=('build_cutlery.py','utensils_v9.py','pivot_v9.py','cover_v9.py','repair_parts.py')
BED_MM=256.


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def mesh_report(mesh):
    assert len(mesh.vertices)>0 and len(mesh.faces)>0,'Empty mesh'
    assert np.isfinite(mesh.vertices).all(),'Non-finite coordinates'
    assert mesh.faces.min()>=0 and mesh.faces.max()<len(mesh.vertices),'Invalid vertex index'
    _,counts=np.unique(mesh.edges_sorted,axis=0,return_counts=True)
    assert np.all(counts==2),'Non-manifold edge incidence'
    assert mesh.is_watertight,'Open mesh'
    assert mesh.is_winding_consistent,'Inconsistent winding'
    assert mesh.volume>0,'Non-positive mesh volume'
    solid=mf.Manifold(mf.Mesh64(np.asarray(mesh.vertices),np.asarray(mesh.faces,dtype=np.uint64)))
    assert solid.status()==mf.Error.NoError,'Mesh kernel rejected export'
    assert len(solid.decompose())==1,'Detached mesh components'
    return {'bounds_mm':mesh.bounds.tolist(),'volume_mm3':float(mesh.volume),
            'vertices':len(mesh.vertices),'triangles':len(mesh.faces),
            'watertight':True,'consistent_winding':True,'invalid_edge_count':0,
            'solid_count':1}


def source_report(root):
    expected,name=(root/'source'/'source.sha256').read_text().strip().split(maxsplit=1)
    name=name.lstrip('*')
    assert Path(name).name==name,'Source manifest must name a local file'
    actual=sha(root/'source'/name)
    assert actual==expected,'Distributed source differs from its recorded checksum'
    result={'file':name,'recorded_source_sha256':expected,'source_copy_sha256':actual}
    return result


def audit_3mf(root,filename,stl_dir,expected):
    path=root/filename
    with ZipFile(path) as archive:
        assert archive.testzip() is None,'ZIP CRC failure'
        names=archive.namelist()
        assert len(names)==len(set(names)),'Duplicate archive members'
        assert {'[Content_Types].xml','_rels/.rels','3D/3dmodel.model'}<=set(names),'Missing required OPC files'
        rels=ET.fromstring(archive.read('_rels/.rels'))
        assert len(rels)==1 and rels[0].get('Target')=='/3D/3dmodel.model','Unexpected package model relationship'
        content_types=ET.fromstring(archive.read('[Content_Types].xml'))
        assert any(x.get('Extension')=='model' and x.get('ContentType')=='application/vnd.ms-package.3dmanufacturing-3dmodel+xml' for x in content_types)
        model_root=ET.fromstring(archive.read('3D/3dmodel.model'))
    assert model_root.get('unit')=='millimeter','Wrong or missing unit'
    objects=model_root.findall('./m:resources/m:object',NS)
    assert len(objects)==len(expected) and {o.get('name') for o in objects}==expected,'Wrong part inventory'
    by_id={o.get('id'):o for o in objects}
    assert len(by_id)==len(expected),'Duplicate object IDs'
    assert {p.stem for p in (root/stl_dir).glob('*.stl')}==expected,'Wrong STL inventory'
    meshes={};reports={}
    for obj in objects:
        name=obj.get('name')
        assert obj.get('type')=='model'
        vertices=np.array([[float(v.get(k)) for k in ('x','y','z')]
                           for v in obj.findall('./m:mesh/m:vertices/m:vertex',NS)])
        faces=np.array([[int(t.get(k)) for k in ('v1','v2','v3')]
                        for t in obj.findall('./m:mesh/m:triangles/m:triangle',NS)])
        mesh=trimesh.Trimesh(vertices,faces,process=True)
        stl_path=root/stl_dir/f'{name}.stl'
        stl=trimesh.load_mesh(stl_path,process=True)
        reports[name]={'3mf':mesh_report(mesh),'stl':mesh_report(stl),'stl_sha256':sha(stl_path)}
        assert mesh.triangles.shape==stl.triangles.shape,f'{name}: STL/3MF triangle count mismatch'
        # Both formats retain exported face order. Compare actual triangle
        # coordinates, not just bounds and volume, allowing float32 STL noise.
        assert np.allclose(mesh.triangles,stl.triangles,rtol=0,atol=2e-5),f'{name}: STL/3MF geometry differs'
        assert abs(float(mesh.bounds[0,2]))<=1e-6,f'{name}: local mesh does not touch Z0'
        meshes[obj.get('id')]=mesh
    build=model_root.findall('./m:build/m:item',NS)
    assert len(build)==len(expected) and {i.get('objectid') for i in build}==set(by_id),'Missing or repeated build item'
    placed={}
    for item in build:
        oid=item.get('objectid');name=by_id[oid].get('name')
        values=np.array([float(x) for x in item.get('transform','1 0 0 0 1 0 0 0 1 0 0 0').split()])
        assert len(values)==12 and np.isfinite(values).all(),'Invalid transform'
        # 3MF row-vector layout: the final three values are translation.
        transform=np.eye(4);transform[:3,:3]=values[:9].reshape(3,3).T;transform[:3,3]=values[9:]
        assert np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-8),'Unexpected scale/shear'
        assert np.linalg.det(transform[:3,:3])>0,'Mirrored build item'
        mesh=meshes[oid].copy();mesh.apply_transform(transform)
        assert np.all(mesh.bounds[0,:2]>=-1e-6) and np.all(mesh.bounds[1,:2]<=BED_MM+1e-6),f'{name}: outside bed'
        assert abs(float(mesh.bounds[0,2]))<=1e-6,f'{name}: transformed part does not touch bed'
        reports[name]['plate_bounds_mm']=mesh.bounds.tolist();placed[name]=mesh
    gaps={}
    for a,b in combinations(placed,2):
        aa,bb=placed[a].bounds[:,:2],placed[b].bounds[:,:2]
        separation=np.maximum(aa[0]-bb[1],bb[0]-aa[1])
        assert np.any(separation>0),f'Overlapping XY bounding rectangles: {a}/{b}'
        gaps[a+'/'+b]=float(np.linalg.norm(np.maximum(separation,0)))
    return {'package_sha256':sha(path),'model_units':'millimeter','object_count':len(objects),
            'bed_mm':[BED_MM,BED_MM],'minimum_xy_bounding_rectangle_gap_mm':min(gaps.values()),
            'objects':reports,'all_stl_triangle_coordinates_match_3mf':True,
            'all_parts_touch_bed':True,'all_footprint_rectangles_disjoint':True}


def audit(root):
    report={'status':'PASS','scope':'Source preservation, exported geometry integrity, millimetre units and independent plate layout checks.',
            'source':source_report(root),'packages':{},
            'not_verified':['Printer or slicer acceptance','Physical fit, strength or durability','Food-contact suitability']}
    for name,(directory,expected) in EXPECTED.items():
        report['packages'][name]=audit_3mf(root,name,directory,expected)
    # The smaller test must use byte-identical full-size hardware.
    shared=['barrel','cap','washer','keeper','cover_key']
    report['mechanism_test_hardware_matches_full_parts']={
        n:sha(root/'print'/f'{n}.stl')==sha(root/'mechanism-test'/f'{n}.stl') for n in shared}
    assert all(report['mechanism_test_hardware_matches_full_parts'].values()),'Test hardware differs from full parts'
    (root/'package-audit.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def rebuild(root):
    report={'status':'PENDING','scope':'Clean temporary-directory build; STL bytes and all 3MF member payloads compared. ZIP timestamps excluded.',
            'copied_build_modules':list(BUILD_MODULES),'module_sha256':{n:sha(root/n) for n in BUILD_MODULES},
            'dependency_versions':{n:importlib.metadata.version(n) for n in ('numpy','trimesh','manifold3d')},
            'stl_comparisons':{},'3mf_payload_comparisons':{}}
    with tempfile.TemporaryDirectory(prefix='cutlery-v9-rebuild-') as temp:
        target=Path(temp)
        for module in BUILD_MODULES:shutil.copy2(root/module,target/module)
        shutil.copytree(root/'source',target/'source')
        result=subprocess.run([sys.executable,str(target/'build_cutlery.py')],cwd=target,
                              capture_output=True,text=True,timeout=180)
        report['builder_exit_code']=result.returncode
        report['builder_output']=result.stdout[-5000:]
        if result.returncode:
            report['status']='FAIL';report['builder_errors']=result.stderr[-5000:]
            (root/'rebuild-validation.json').write_text(json.dumps(report,indent=2)+'\n')
            raise AssertionError('Clean build failed; inspect rebuild-validation.json')
        for filename,(directory,expected) in EXPECTED.items():
            for name in sorted(expected):
                relative=Path(directory)/f'{name}.stl'
                a,b=sha(root/relative),sha(target/relative)
                report['stl_comparisons'][str(relative)]={'delivered_sha256':a,'rebuilt_sha256':b,'identical_bytes':a==b}
            with ZipFile(root/filename) as old,ZipFile(target/filename) as new:
                assert set(old.namelist())==set(new.namelist()),'Rebuilt 3MF member inventory changed'
                checks={}
                for member in old.namelist():
                    a,b=old.read(member),new.read(member)
                    checks[member]={'delivered_sha256':hashlib.sha256(a).hexdigest(),
                                    'rebuilt_sha256':hashlib.sha256(b).hexdigest(),'identical_bytes':a==b}
                report['3mf_payload_comparisons'][filename]=checks
        report['status']='PASS' if (all(x['identical_bytes'] for x in report['stl_comparisons'].values())
            and all(v['identical_bytes'] for package in report['3mf_payload_comparisons'].values() for v in package.values())) else 'FAIL'
    (root/'rebuild-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    assert report['status']=='PASS','Rebuilt geometry differs; inspect rebuild-validation.json'
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=ROOT)
    parser.add_argument('--rebuild',action='store_true',help='Also build from only copied sources in a temporary directory')
    args=parser.parse_args();root=args.root.resolve()
    report=audit(root)
    summary={'package_audit':report['status'],'packages':{
        name:{'objects':r['object_count'],'minimum_plate_gap_mm':r['minimum_xy_bounding_rectangle_gap_mm']}
        for name,r in report['packages'].items()}}
    if args.rebuild:summary['clean_rebuild']=rebuild(root)['status']
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
