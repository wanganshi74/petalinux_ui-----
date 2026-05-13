# 纯 DRM 第一页

这个工程现在只保留一条线：

- `libdrm + /dev/dri/cardX + dumb buffer`
- 不用 LVGL
- 不用触摸
- 只显示第一页

当前真正会参与编译的文件只有：

- `src/drm_page1.c`
- `src/power_model.c`
- `src/power_model.h`
- `Makefile`

## 编译

在 Zynq 的 Linux 交叉编译环境里执行：

```bash
make UI_DRM_CARD=/dev/dri/card0
```

如果 LCD 实际挂在 `card1`：

```bash
make clean
make UI_DRM_CARD=/dev/dri/card1
```

生成文件：

- `./drm_page1`

## 运行

板端运行：

```bash
chmod +x ./drm_page1
./drm_page1
```

如果要明确走 HDMI 或另一个 DRM 节点，也可以：

```bash
./drm_page1 hdmi
```

或者直接传设备路径：

```bash
./drm_page1 /dev/dri/card1
```

## 依赖

板端或 sysroot 里需要：

- `libdrm`
- `xf86drm.h`
- `xf86drmMode.h`
- `pkg-config`

## 当前效果

程序会直接把第一页画出来，包含：

- 顶部标题栏
- 左侧功率信息区
- 右上频率和相位卡片
- 右侧关键参数区
- 底部 `MAIN / FFT / HOLD` 导航条

数值来自 `power_model.c` 里的演示数据，每秒刷新一次。
