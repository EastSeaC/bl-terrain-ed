# 可复现的编辑地形重生成器

本实现替代原项目存在缺陷的 C# 生成路径。直接从 `scene.xscene` 与 `terrain.bin`
读取高度、法线、全部材质权重，重新序列化与压缩；不接受旧 ed 作为生成输入。
原 C# 源码和 exe 未改动，不能通过继续运行原 exe 获得此修复。

## 已确认的根因

已对工作区 `EWModResources/ed-extractor-1.1.0-win-x64/Terrain.dll` 的
`Scene.TerrainEd`、`Scene.Chunks.NrmlChunk` 用 ilspycmd 反编译，确认发布 DLL
包含与本项目源码相同的错误；反编译证据保存在
`EWModResources/NavMeshWork/ed-extractor-decompiled`。

- `TerrainEd.WriteFile` 将 `GetStorageSize()` 的预估容量写入头部，未使用实际流长度。
  该估算包含 PHYM 的四通道数据，而实际写入的是单通道零数组。Normandy 差值恰好为
  `16×16×257×257×3×4 = 202,902,528` 字节，与审计发现完全一致。
- `BuildEditData` 在写 float4 时读取 `nrmlNode[k..k+3]`，却每次只令 k 增加 1；
  应按每顶点 4 个 float 前进，旧实现造成循环滑窗错位。
- `NrmlChunk.ReadPNG` 使用 `byte/255 + 0.5`，而非将 XY 映射到 `[-1,1]` 并恢复正 Z、归一化。
  `range % TerrainX` 也不是当前像素列号，且节点和像素转置没有正确实现。

无需继续反编译引擎 Native DLL 即可确认上述生成器缺陷。容器和节点布局参考现有
`BannerlordEternalWorld/Tools/NavMesh/terrain_ed.py` 及 Aragon 原版数据交叉验证。

## 使用

以下命令在 `EternalWorldX` 工作区根目录执行。需要 numpy、Pillow、lz4；当前工作区
已有这些依赖，脚本自动使用 `EWModResources/NavMeshWork/python-deps` 中的 lz4。
另附 requirements.txt 供其他环境安装；脚本仍依赖本工作区的共享格式读取模块。

```powershell
python bl-terrain-ed/python/regenerate.py --scene EWModResources/SceneObj/ew_normandy_v13 --out EWModResources/NavMeshWork/my-new-version/terrain_ed.bin
python bl-terrain-ed/python/verify.py --new EWModResources/NavMeshWork/my-new-version/terrain_ed.bin --reference EWModResources/SceneEditData/ew_normandy_v13/terrain_ed.bin --report EWModResources/NavMeshWork/my-new-version/verification.json
python -m unittest discover -s bl-terrain-ed/python -p test_regenerate.py -v
```

输出路径必须不存在，防止覆盖原图。每次生成伴随 `.generation.json`，记录输入哈希、
输出哈希、实际长度及法线来源。`verify.py` 严格验证新文件头，参考旧文件时才允许旧头长度异常。
生成不会修改 `scene.xscene`、`terrain.bin`，也不会部署。

## 重建规则

- 高度：16bit HGHT，按场景 min/max 还原世界高度，节点和像素顺序分别转置。
- 权重：逐节点解出 WGHT 的 16 个槽位，恢复实际存在的 1..15 号层；保持权重原值，
  不归一化、不改成常量。遇到槽 15 有数据时拒绝生成，避免无依据地丢弃第 16 层。
- 法线 auto：通常解码运行时 NRML；如果 NRML 的 XY 全图为常量而高度有起伏，
  改用跨节点共享高度网格的中心差分，世界边界用单边差分。可显式指定
  `--normals nrml` 或 `--normals heights`。
- 保留四季掩码；a2 为零，与此处 Aragon 原版一致，但其语义尚未确认。
- 实际流长度写入压缩头，标准 LZ4 raw block；保留字段写零，不复制旧内存地址。
- 当前支持已验证的灰度权重、法线/高度/权重相同分辨率；高度重算法线要求节点分辨率统一。
  不满足时明确失败，避免无提示的重采样或丢层。

## 本次实测

产物在 `EWModResources/NavMeshWork/terrain-ed-fixed/`：

| 项目 | Aragon 原版对拍 | Normandy V13 旧产物对拍 |
|---|---:|---:|
| 节点数 | 169 | 256 |
| 核验材质层块数 | 906 | 1837 |
| 高度最大差 | 0.002417 m | 0.00001526 m |
| 权重最大差 | 0.00392156（8bit 量化范围） | **0** |
| 法线单位长度最大偏差 | 8.66e-8 | 5.01e-8 |
| 高度接缝最大差 | 0 | 0 |
| 法线接缝最大分量差 | 0 | 0 |
| 解压长度与头部 | 一致 | **891,184,496 字节，一致** |

Aragon 重建法线与原版夹角中位 0.363°、95 分位 0.609°。
Normandy 新文件 292,656,091 字节，比旧文件大，主要因为法线不再是重复的错误常量；
此处使用标准快速 LZ4，未做压缩率优化。生成后的法线是高度几何重建结果，不能称为恢复了
源作者的原始浮点法线；高度及权重也只能恢复运行时量化精度。

5 项回归覆盖非对称坡面方向、跨节点中心差分、权重转置、向量序列化与常量 NRML 检测。
完整逐节点统计见各产物目录的 verification.json。通过静态检查尚不代表已通过编辑器
加载、保存与重开测试，本次未替换模块内文件。
