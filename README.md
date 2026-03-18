# Usage

把 OpenCV 删掉了，仅适用于 Windows 平台，其他平台没有给 Assimp 库，文件，自行下载配置。Linux 平台在另一个仓库里面

使用

```cmd
mkdir build
cd build
cmake ../
cmake --build . --config Release
```

找到生成的可执行文件，切换至其所在目录为工作目录，下面是测试命令：

```cmd
./ModelTransformer -i test.obj -d 10 -o test.dxf -t bdxf -f ascii -r 4
```

test.obj 在仓库根目录提供。需要自行复制。

具体命令的意义参考 `main.cpp`。

`-d` 表示转为点云时的插值间隔

`-r` 表示点云转为网格时的分辨率

这两个数据目前测试能够生成。

某些数据可能会导致体积过大，数字溢出，触发异常（理论上讲这样的话命令行会出现一个 `Error: ` 开头的报错，这是包裹整个 `main` 函数的 `try-catch` 组）

还有一种情况，没有任何提示，直接退出。这种情况的触发原因尚不明确。