"""从运行时地形重新生成编辑地形；不读取或修补已有 ed 文件。"""
from pathlib import Path
import argparse
import hashlib
import io
import json
import os
import struct
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'BannerlordEternalWorld/Tools/NavMesh'))
sys.path.insert(0, str(ROOT / 'EWModResources/NavMeshWork/python-deps'))
import lz4.block
import terrain_ed as fmt

# 独立标准 LZ4，避免纯 Python 编码器在大图上耗费数分钟。
fmt.lz4_decode = lambda data, max_out: lz4.block.decompress(data, uncompressed_size=max_out)


def read_weights(payload, count):
    """WGHT 每节点 16 槽：存在标志 + PNG；节点序为 y,x，像素转为 x,y。"""
    off, nodes = 0, []
    for _ in range(count):
        layers = []
        for slot in range(16):
            present, = struct.unpack_from('<I', payload, off)
            off += 4
            if present not in (0, 1):
                raise ValueError('WGHT 非法存在标志')
            if present:
                size, = struct.unpack_from('<I', payload, off)
                off += 4
                image = Image.open(io.BytesIO(payload[off:off+size]))
                off += size
                pixels = np.asarray(image)
                if pixels.dtype != np.uint8 or pixels.ndim != 2:
                    raise ValueError('仅支持已验证的 8bit 灰度权重 PNG')
                if slot == 15:
                    raise ValueError('槽 15 有数据：编辑格式仅验证了层号 1..15，拒绝丢弃')
                layers.append((slot+1, np.ascontiguousarray(pixels.T, dtype='<f4') / np.float32(255)))
        nodes.append(layers)
    if off != len(payload):
        raise ValueError(f'WGHT 未完整消费：{off}/{len(payload)}')
    return nodes


def stitch(heights, nx, ny):
    """拼世界坐标 [y,x] 高度，检查共享边，供跨节点中心差分使用。"""
    sizes = {h.shape for h in heights}
    if len(sizes) != 1:
        raise ValueError('高度重算法线暂不支持混合节点分辨率；拒绝静默近似')
    h, w = heights[0].shape
    grid = np.full((ny*(h-1)+1, nx*(w-1)+1), np.nan, dtype='<f4')
    for k, tile in enumerate(heights):
        iy, ix = divmod(k, nx)
        target = grid[iy*(h-1):iy*(h-1)+h, ix*(w-1):ix*(w-1)+w]
        overlap = np.isfinite(target)
        if overlap.any() and np.max(np.abs(target[overlap]-tile[overlap])) > 0.01:
            raise ValueError('源高度节点共享边不一致')
        target[:] = tile
    return grid


def height_normals(grid, ix, iy, width, height, node_size):
    """读一圈邻域，避免对每个节点单独差分造成法线接缝；输出 [x,y,4]。"""
    x, y = ix*(width-1), iy*(height-1)
    left, top = max(0,x-1), max(0,y-1)
    patch = grid[top:min(grid.shape[0],y+height+1),left:min(grid.shape[1],x+width+1)]
    gy, gx = np.gradient(patch.astype(np.float64), node_size/(height-1), node_size/(width-1))
    crop = (slice(y-top,y-top+height),slice(x-left,x-left+width))
    xyz = np.stack((-gx[crop],-gy[crop],np.ones((height,width))),axis=-1)
    xyz /= np.linalg.norm(xyz,axis=-1,keepdims=True)
    result = np.zeros((height,width,4),dtype='<f4')
    result[...,:3] = xyz
    return np.ascontiguousarray(result.transpose(1,0,2))


def choose_normals(mode, nrml, heights):
    if mode != 'auto':
        return mode, '用户指定'
    if not nrml:
        return 'heights', 'NRML 缺失'
    # 常量方向配合起伏高度，不能提供可信的逐点地形法线。
    lo = np.min([a[...,:2].min(axis=(0,1)) for a in nrml],axis=0)
    hi = np.max([a[...,:2].max(axis=(0,1)) for a in nrml],axis=0)
    span = max(float(a.max()) for a in heights)-min(float(a.min()) for a in heights)
    if np.array_equal(lo,hi) and span > 0.01:
        return 'heights', 'NRML 的 XY 全图为常量，但高度存在起伏'
    return 'nrml', 'NRML 含空间变化'


def write_node(stream, ix, iy, masks, normals, heights, layers):
    w, h = heights.shape
    if normals.shape != (w,h,4) or any(a.shape != (w,h) for _,a in layers):
        raise ValueError('法线/权重/高度分辨率不一致；当前实现拒绝重采样')
    mask = sum(1 << (i-1) for i,_ in layers)
    if any(m & ~mask for m in masks):
        raise ValueError('场景层掩码引用了 WGHT 中不存在的层')
    # 保留场景四季掩码，不将四份掩码强制改成相同值。
    stream.write(struct.pack('<6I2I',ix,iy,*masks,w,h))
    stream.write(normals.astype('<f4',copy=False).tobytes())
    stream.write(struct.pack('<2I',w,h))
    stream.write(bytes(w*h*4))  # a2 与已验证 Aragon 一致；语义尚未确认。
    stream.write(struct.pack('<32I',*([w,h]*16)))
    stream.write(heights.astype('<f4',copy=False).tobytes())
    stream.write(struct.pack('<I',len(layers)))
    for layer,weights in layers:
        stream.write(struct.pack('<I',layer))
        stream.write(weights.astype('<f4',copy=False).tobytes())


def regenerate(scene, output, mode='auto'):
    scene, output = Path(scene), Path(output)
    if output.exists():
        raise FileExistsError(f'拒绝覆盖：{output}')
    sc = fmt.scene_terrain(scene/'scene.xscene')
    nx,ny = sc['node_dim_x'],sc['node_dim_y']
    tb = fmt.TerrainBin(scene/'terrain.bin')
    pngs = tb.png_segments('HGHT')
    if len(pngs) != nx*ny:
        raise ValueError('HGHT 节点数与场景不一致')
    heights = [np.asarray(sc['min_height']+(sc['max_height']-sc['min_height'])*a.astype(np.float64)/65535,dtype='<f4') for a in pngs]
    nrml = tb.png_segments('NRML') if 'NRML' in tb.chunks else []
    if nrml and (len(nrml) != nx*ny or any(a.ndim!=3 or a.shape[-1]!=4 for a in nrml)):
        raise ValueError('NRML 节点数/PNG 格式不符')
    mode,reason = choose_normals(mode,nrml,heights)
    grid = stitch(heights,nx,ny) if mode == 'heights' else None
    layers = read_weights(tb.payload('WGHT'),nx*ny)
    xml = ET.parse(scene/'scene.xscene')
    nodes = {(int(n.get('idx')),int(n.get('idy'))):n for n in xml.findall('.//terrain/nodes/node')}
    if len(nodes) != nx*ny:
        raise ValueError('场景节点数量不符')
    output.parent.mkdir(parents=True,exist_ok=True)
    print(f'{scene.name}: {nx}x{ny}, normals={mode}: {reason}',flush=True)
    fd,tmp = tempfile.mkstemp(prefix='terrain-ed-',suffix='.raw',dir=output.parent)
    try:
        with os.fdopen(fd,'wb') as stream:
            stream.write(struct.pack('<2I',nx,ny))
            for ix in range(nx):
                for iy in range(ny):
                    k = iy*nx+ix
                    h = np.ascontiguousarray(heights[k].T)
                    normals = (height_normals(grid,ix,iy,*h.shape,sc['node_size']) if mode=='heights'
                               else fmt.normals_from_nrml(nrml[k].transpose(1,0,2)))
                    node = nodes[ix,iy]
                    masks = tuple(int(node.get('layer_is_used_mask_'+s)) for s in ('summer','fall','winter','spring'))
                    write_node(stream,ix,iy,masks,normals,h,layers[k])
        # 使用实际写入长度，绝不使用预分配容量估算文件头。
        raw = Path(tmp).read_bytes()
        fmt.walk(raw)
        comp = lz4.block.compress(raw,store_size=False)
        plen = 40+len(comp)
        header = b'OK01'+struct.pack('<4Q',plen,2,len(raw),plen)+bytes(16)
        with output.open('xb') as stream:
            stream.write(header)
            stream.write(comp)
        report = dict(scene=str(scene.resolve()),output=str(output.resolve()),normal_source=mode,normal_reason=reason,
                      nodes=[nx,ny],raw_bytes=len(raw),file_bytes=output.stat().st_size,
                      sha256=hashlib.sha256(header+comp).hexdigest(),
                      inputs={name:hashlib.sha256((scene/name).read_bytes()).hexdigest() for name in ('terrain.bin','scene.xscene')})
        output.with_suffix('.generation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
        return report
    finally:
        Path(tmp).unlink(missing_ok=True)


if __name__=='__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--normals',choices=('auto','nrml','heights'),default='auto')
    a=p.parse_args()
    regenerate(a.scene,a.out,a.normals)
