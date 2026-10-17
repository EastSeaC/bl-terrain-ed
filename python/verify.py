"""新产物严格读取，逐节点对拍原版/旧产物的高度与全部权重，报告法线与接缝。"""
import argparse
import json
from pathlib import Path
import struct
import numpy as np
import regenerate as r


def load(path,strict=True):
    d=Path(path).read_bytes()
    assert d[:4]==b'OK01'
    plen,ctype,unc,comp=struct.unpack_from('<4Q',d,4)
    assert ctype==2 and plen==comp and len(d)==12+plen
    raw=r.lz4.block.decompress(d[52:],uncompressed_size=unc)
    if strict:
        assert len(raw)==unc,(len(raw),unc)
    return raw,r.fmt.walk(raw),dict(declared=unc,actual=len(raw))


def verify(new,reference):
    raw,(nx,ny,nodes),header=load(new)
    old,(ox,oy,onodes),oldheader=load(reference,False)
    assert (nx,ny)==(ox,oy)
    max_height=max_weight=max_unit=max_seam=0.
    angles=[];xyz_min=np.full(3,np.inf);xyz_max=np.full(3,-np.inf)
    heightmap={};normalmap={};layer_count=0
    for n,o in zip(nodes,onodes):
        assert (n['ix'],n['iy'],n['mask'])==(o['ix'],o['iy'],o['mask'])
        shape=(n['gw3'],n['gh3'])
        a=r.fmt.f32(raw,n['o_h'],shape);b=r.fmt.f32(old,o['o_h'],shape)
        max_height=max(max_height,float(np.abs(a-b).max()))
        assert [li for li,_,_ in n['layers']]==[li for li,_,_ in o['layers']]
        for (_,off,_),(_,oo,_) in zip(n['layers'],o['layers']):
            w=r.fmt.f32(raw,off,shape);ow=r.fmt.f32(old,oo,shape)
            assert np.isfinite(w).all() and w.min()>=0 and w.max()<=1
            max_weight=max(max_weight,float(np.abs(w-ow).max()));layer_count+=1
        normals=r.fmt.f32(raw,n['o_norm'],(*shape,4))
        assert np.isfinite(normals).all() and (normals[...,3]==0).all()
        xyz=normals[...,:3].astype(np.float64)
        lengths=np.linalg.norm(xyz,axis=-1)
        max_unit=max(max_unit,float(np.abs(lengths-1).max()))
        xyz_min=np.minimum(xyz_min,xyz.min(axis=(0,1)))
        xyz_max=np.maximum(xyz_max,xyz.max(axis=(0,1)))
        oldxyz=r.fmt.f32(old,o['o_norm'],(*shape,4))[...,:3].astype(np.float64)
        cosine=(xyz*oldxyz).sum(-1)/(lengths*np.linalg.norm(oldxyz,axis=-1))
        angles.append(np.degrees(np.arccos(np.clip(cosine,-1,1))).ravel())
        heightmap[n['ix'],n['iy']]=a
        normalmap[n['ix'],n['iy']]=normals
    normal_seam=0.
    for (ix,iy),h in heightmap.items():
        normals=normalmap[ix,iy]
        for other,edge,opposite in (((ix+1,iy),(-1,slice(None)),(0,slice(None))),((ix,iy+1),(slice(None),-1),(slice(None),0))):
            if other in heightmap:
                max_seam=max(max_seam,float(np.abs(h[edge]-heightmap[other][opposite]).max()))
                normal_seam=max(normal_seam,float(np.abs(normals[edge]-normalmap[other][opposite]).max()))
    assert max_unit<2e-7 and max_seam==0
    assert max_height<0.003 and max_weight<0.0041
    angles=np.concatenate(angles)
    result=dict(new=str(Path(new).resolve()),reference=str(Path(reference).resolve()),new_header=header,reference_header=oldheader,
                nodes=len(nodes),layer_blocks_compared=layer_count,max_height_difference_m=max_height,
                max_weight_difference=max_weight,max_normal_unit_error=max_unit,height_seam_max_m=max_seam,
                normal_seam_component_max=normal_seam,normal_xyz_min=xyz_min.tolist(),normal_xyz_max=xyz_max.tolist(),
                normal_angle_to_reference_median_deg=float(np.median(angles)),normal_angle_to_reference_p95_deg=float(np.percentile(angles,95)),
                static_checks_passed=True,engine_load_tested=False)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--new',required=True);p.add_argument('--reference',required=True);p.add_argument('--report',required=True)
    a=p.parse_args();result=verify(a.new,a.reference)
    Path(a.report).write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2))
