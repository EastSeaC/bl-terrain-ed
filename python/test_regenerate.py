"""非对称坡面、跨节点接缝、权重方向与容器长度回归。"""
import io
import struct
import unittest
import numpy as np
from PIL import Image
import regenerate as r


class RegenerationTests(unittest.TestCase):
    def test_asymmetric_plane_world_axes(self):
        y,x = np.mgrid[:7,:9]
        grid = (2*x+3*y+10).astype('<f4')
        # 4x3 米节点、5x4 样点要求等间距；此处直接用正方形节点验证方向。
        normals = r.height_normals(grid,0,0,5,5,4)
        expected = np.array([-2,-3,1])/np.sqrt(14)
        np.testing.assert_allclose(normals[...,:3],np.broadcast_to(expected,(5,5,3)),atol=1e-7)
        self.assertTrue(np.all(normals[...,3]==0))

    def test_cross_node_normals_share_central_difference(self):
        y,x = np.mgrid[:5,:9]
        grid = (x*x+0.3*y*y).astype('<f4')
        left = r.height_normals(grid,0,0,5,5,4)
        right = r.height_normals(grid,1,0,5,5,4)
        np.testing.assert_array_equal(left[-1],right[0])
        expected = np.array([-8,-1.2,1]); expected /= np.linalg.norm(expected)
        np.testing.assert_allclose(left[-1,2,:3],expected,atol=1e-7)

    def test_weight_png_transpose_and_layer_ids(self):
        source=np.array([[0,51,102],[153,204,255]],dtype=np.uint8)
        png=io.BytesIO(); Image.fromarray(source).save(png,format='PNG'); data=png.getvalue()
        payload=b''.join(struct.pack('<I',1)+struct.pack('<I',len(data))+data if i==2 else struct.pack('<I',0) for i in range(16))
        layers=r.read_weights(payload,1)[0]
        self.assertEqual(layers[0][0],3)
        np.testing.assert_array_equal(layers[0][1],source.T.astype('<f4')/np.float32(255))
        with self.assertRaises(ValueError):
            r.read_weights(payload+b'junk',1)

    def test_container_and_vector_stride(self):
        stream=io.BytesIO(); stream.write(struct.pack('<2I',1,1))
        h=np.arange(9,dtype='<f4').reshape(3,3)
        n=r.height_normals(h.T,0,0,3,3,2)
        r.write_node(stream,0,0,(1,1,1,1),n,h,[(1,np.ones((3,3),dtype='<f4'))])
        raw=stream.getvalue(); nx,ny,nodes=r.fmt.walk(raw)
        self.assertEqual((nx,ny),(1,1))
        np.testing.assert_array_equal(r.fmt.f32(raw,nodes[0]['o_norm'],(3,3,4)),n)
        comp=r.lz4.block.compress(raw,store_size=False)
        self.assertEqual(r.lz4.block.decompress(comp,uncompressed_size=len(raw)),raw)

    def test_auto_rejects_flat_nrml_on_hilly_terrain(self):
        n=[np.full((3,3,4),127,dtype=np.uint8)]
        self.assertEqual(r.choose_normals('auto',n,[np.arange(9).reshape(3,3)])[0],'heights')


if __name__=='__main__':
    unittest.main()
