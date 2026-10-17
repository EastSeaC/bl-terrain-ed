## 修正后的 Python 重生成入口（2026-10-16）

当前 EW 工作区请使用 [python/regenerate.py](python/regenerate.py)。它从
`scene.xscene + terrain.bin` 重建整个 `terrain_ed.bin`，不会读取或修补旧 ed。
原 C# 源码和发布的 `Terrain.dll` 保留为参考；其生成路径仍有已确认的问题，
不要继续使用原 exe 生成正式编辑数据。详见 [python/README.md](python/README.md)。

```powershell
python bl-terrain-ed/python/regenerate.py --scene EWModResources/SceneObj/ew_normandy_v13 --out EWModResources/NavMeshWork/my-new-version/terrain_ed.bin
```

以下为原版 C# 工具说明：

Source path can refer to a SceneObj's directory or to a parent directory that contains multiple SceneObj folders.
\
Output path should be a parent folder as it will create a sub folder for each scene's terrain edit data

Use the exe and it will prompt you for the paths or run it via the cmd line and you can pass the paths as arguments.

```
Terrain.exe C:\Users\Admin\Desktop\SourceFolder C:\Users\Admin\Desktop\OutputFolder
```

Probably best not to do a lot of scenes at once, as they can get pretty bloated.
