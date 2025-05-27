# kmz_decode

> **对 KMZ（KML + WPML）飞行路线文件的解析与重构工具**  
> Parse & rebuild drone wayline *KMZ* packages that contain `template.kml` and `waylines.wpml`.

---

## ✨ 项目简介 | Overview

DJI/Autel 等无人机在导出航线时，会得到一个扩展名为 `.kmz` 的压缩包——它本质上是一个 **ZIP**，内部至少包含两部分：  

| 组件            | 作用 | 参考 |
|-----------------|------|------|
| `template.kml`  | 航线的业务属性、可视化信息 | WPML 规范 :contentReference[oaicite:0]{index=0} |
| `waylines.wpml` | 具体的航点、航向、云台动作等指令，使用 **WPML (WayPoint Markup Language)** 编写 | DJI/Autel 文档、社区讨论 :contentReference[oaicite:1]{index=1} |

本项目提供一个轻量级 **C ++17** 命令行工具，**预计**完成以下工作：

* **解包 / 打包** KMZ 文件  
* **解析** `waylines.wpml` 与 `template.kml`，转为易于处理的内部结构  
* **修改并重新生成** 航线（例如批量平移、反转航点、批量修改云台动作）  
* **导出** 新的 KMZ，以便重新导入 DJI Pilot / FlightHub 2 等软件  
* **示例** 展示如何在自动化流水线中批量处理航线

> 本仓库仅做技术研究与学习，请在遵守当地法规、生产厂商条款的前提下使用。

---

## 🏗️ 构建 | Build

> 已使用 **CMake** 管理依赖，无须额外安装第三方库（`third_party/` 目录已内置依赖）。

```bash
git clone https://github.com/xiayang-cmd/kmz_decode.git
cd kmz_decode
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)

# 生成的可执行文件
./kmz_decode --help
