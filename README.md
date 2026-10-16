# dart_description

ROS 2 Jazzy 飞镖系统描述包。实际运行只加载镖架、双目相机和离轨点 TF；场景分析按需加载 RMUC2027 彩色地图、目标和舱门。两种模式共用机械描述，地图模型保留 STEP 材质和人工确认的黑色面。

## 启动

```bash
colcon build --packages-select dart_description --symlink-install
source install/setup.bash
# 实际运行：默认 robot 模式，只启动 robot_state_publisher
ros2 launch dart_description description.launch.py

# 仅查看镖架、相机和视线，GUI 提供 yaw 状态
ros2 launch dart_description view_description.launch.py mode:=robot

# 完整场景分析：view_description 默认 scene 模式
ros2 launch dart_description view_description.launch.py team:=red
```

实际运行以 `dart_base_link` 为根，共 8 个 link、7 个 joint；唯一活动关节为 `launcher_yaw_joint`。不加载场地、目标、队伍别名或场地状态发布器。控制端必须向 `/joint_states` 发布实际 yaw（弧度），才能得到基座到相机、离轨点的完整动态 TF；运行入口不会自动发布零位。相机内部与相机到离轨点的固定关系仍可独立使用。

场景模式中，`team:=blue` 切换己方镖架和目标别名；场地原点不变。GUI 提供两侧基地滑轨、两侧舱门和镖架 yaw 滑块。
当前使用保色轻量显示模型：场景约 328.8 万个三角面，共用 48 份 DAE、约 120.5 MiB（复用前 281.3 MiB）。唯一存储几何约 147.7 万面，场景实际显示面数不变。高精度母版保留在外部资产目录，关节和目标坐标仍由原始 CAD 定义。

仅发布描述和场地机构 TF：

```bash
ros2 launch dart_description description.launch.py mode:=scene team:=red
ros2 param set /scene_joint_state_publisher blue_base_slide_m 0.28
ros2 param set /scene_joint_state_publisher red_gate_open_m 1.2
```

仅场景模式创建场地状态节点，默认发布两条滑轨和两扇门的可视化位置，不发布 `launcher_yaw_joint`。
镖架 yaw 由实际控制链或 GUI 提供；没有 yaw 状态时，相机到场地的 TF 不完整。
有实际场地机构状态源时，使用 `start_scene_state_publisher:=false`，避免多个发布者控制同一关节。
RViz 启动入口会自动禁用场地状态节点，由 GUI 统一发布关节状态。
辅助显示仅保留左右相机视锥边线和光轴；默认关闭视锥填充面，不再发布基地包围框或水平地面网格。CAD 场地模型完整保留。

`team` 只影响场景分析。安装偏移、行程和尺寸直接定义在 URDF 模块中，不通过 launch 或 xacro 参数覆盖。不再读取 `site_file` 或基地现场偏移。
双目标定数据不由 TF 替代；相机安装外参应以实测标定为准。视锥显示不执行遮挡检测。
本包独立维护场景，不提供主项目的旧接口适配。相机与镖架内部安装外参保留。

## 场地坐标

`field_link` 原点为红方朝蓝方看时的场内右下角：`+x` 红→蓝、`+y` 向左、`+z` 竖直向上。
原点依据规则 28 m × 15 m 场内尺寸与 CAD 中轴建立，角点地面高度为 `z=0`。
保留 CAD 的 1.5° 横坡，15 m 坡面宽度的水平投影约为 14.99486 m。参考网格为水平面，不代表实际地形。
这属于工程坐标基准，尚未进行实地测量校验。

`metadata/field_alignment_2027.json` 中的 `field_from_cad` 将 CAD 世界米坐标变换到场地坐标。
各 DAE 的装配原点与内部节点矩阵保留，由 URDF 的 visual origin 一次性放置。
旧地图点坐标可用 `(y_old, 15 - x_old, z_old)` 改写轴定义，但这不是旧赛季到新赛季的场景配准。

## 机构与目标

| 机构 | 坐标/关节 | 定义 |
| --- | --- | --- |
| 两侧基地主体 | `red_base_link` / `blue_base_link` | CAD 原点，x 朝场内、y 左、z 上 |
| 两侧导轨 | `red_rail_link` / `blue_rail_link` | 固定连接，CAD 导轨中心参考点 |
| 两侧基地目标 | `red_base_target_slide_joint` / `blue_base_target_slide_joint` | 沿所属基地 local +y，范围 ±0.28 m；名称不随 team 改变 |
| 两侧舱门 | `red_dart_gate_joint` / `blue_dart_gate_joint` | 沿 CAD 倾斜导轨直线运动；0 为下方关闭状态，正值上移开门；1.2 m 到达由 CAD 接触面确定的上方打开端 |
| 前哨站顶部 | `<side>_outpost_target_link` | 固定目标；中部旋转装甲暂为静态显示 |

基地行程依据规则 V1.0.0 印刷页 111。舱门导轨方向依据 CAD，1.2 m 行程由用户确认。
CAD 原位不是端点。根据定义 `0:1:1:31` 中运动支架面 16284 与固定框架下表面 4541 的接触，打开端为 CAD 原位沿导轨上移 53.25 mm；向下退 1.2 m 得到关闭零位。原始 BRep 面重合面积和实体内外检查见 `metadata/gate_contact.json`。生成脚本从源面重新计算偏移，不使用目测 55 mm。修改关节 `limit upper` 不会自动移动关闭零位。
舱门行程直接修改 `urdf/modules/dart_stations.xacro` 中两侧舱门关节的 `limit upper`；场景状态节点从 URDF 读取限位。离线 CAD 生成器的行程定义仍为 1.2 m，若改变机械行程，需同步修改生成规则并重新确定关闭零位。

绿灯参考点来自 CAD 发光面包围中心；装甲参考点与法向来自外侧接触面：基地定义 `0:1:1:64` 面 140、前哨站定义 `0:1:1:16` 面 17250。装甲局部 +x 指向外部，y/z 位于接触面内。
这是几何参考，不等同于经过标定的光学中心。具体面号与依据见 `metadata/rm2027_mechanisms.json`。
装甲计算区域继续使用可配置的 100 mm × 100 mm 方形；只保留计算用 collision，不叠加旧版 box/cylinder 显示。

包内目标选择别名始终指向对方基地：`base_link`、`rail_origin_link`、`dart_detection_module_link`、
`green_light_link`、`armor_link`。`base_link` 直接连接所选对方基地，不再设置 nominal 中间坐标系。
前哨站别名为 `outpost_target_link`、`outpost_green_light_link`、`outpost_armor_link`。
所有原有相机 optical frame 名称保持不变。

## 镖架安装

镖架安装基准为用户选定的舱底 A 面（CAD 定义 `0:1:1:31` 的面 71）。原点取完整面的包围中心，z 沿朝上的面法向，x 沿发射站前向在面上的投影。红蓝共用同一局部基准。
相对发射站的基准位置为 `[-0.1385, 0, 0.055]` m，姿态约为零；已移除原上部面造成的 15° 俯仰。该基准随 CAD 发射站的场地横坡放置，不代表绝对水平。安装修正直接编辑以下文件：

| 内容 | 文件与位置 |
|---|---|
| 镖架相对 A 面的安装偏移 | `urdf/modules/targets.xacro` → `dart_pedestal_to_base` 的 `origin xyz/rpy` |
| 舱门行程 | `urdf/modules/dart_stations.xacro` → 两侧 `dart_gate_joint` 的 `limit` |
| 目标装甲板尺寸及厚度 | `urdf/modules/targets.xacro` → 两处 `armor_link` 的 `collision/box`，厚度改变时同步调整 collision origin |
| 飞镖离轨点位置和俯仰角 | `urdf/modules/launcher.xacro` → `launcher_to_dart_release` 的 `origin` |
| 相机安装外参 | `urdf/modules/stereo_camera.xacro` → 对应安装关节的 `origin` |

平移单位米，角度弧度。离轨 pitch 保持 -32°（-0.5585053606381855 rad）。修改后重新启动描述节点生效。xacro 仅保留模块包含、阵营选择及角度表达式，不再提供几何参数覆盖。离线生成器保留源文件中的镖架安装偏移、离轨点和相机链；场地与目标几何仍按生成脚本重建。

## 几何 API

```python
from target_geometry import Geometry

g = Geometry(team="red")
base_green = g.green_position(slide=0.0, reference="dart_base_link")
outpost_green = g.target_position("outpost", "green", reference="dart_base_link")
base_surface = g.armor_surface(slide=0.0)
outpost_surface = g.outpost_armor_surface()
opened_gate = g.pose("red_dart_gate_link", {"red_dart_gate_joint": 1.2})
```

`pose` 接受关节位置字典，统一检查关节名称、有限数值及 URDF 限位；未指定的关节按零位计算。`green_position`、`armor_surface` 等便捷方法也检查滑轨与 yaw 范围。`target_yaw` 返回所选参考系 XY 平面上的目标方位，不是带弹道补偿的控制指令。

`Surface.coordinates(point)` 返回装甲板平面内的 u/v 和有符号法向距离；`contains` 检查有限矩形内外，包含边界并消除浮点舍入误差。当前基地与前哨站均按配置中的 100×100 mm 矩形分析，前哨站复用基地尺寸；这不是对实际有效区域尺寸的独立校验。

该库读取完整场景，可独立进行离线计算，无需启动 ROS 节点或 RViz；不读取实时 `/joint_states`。分析非零关节状态时，使用 `pose(..., joint_positions=...)` 或带 `slide/launcher_yaw` 的 `link_pose`、`position` 接口。
离线调用时需传入与 launch 一致的 `team`，默认红方。完整坐标树由 URDF 提供，不在 API 中复制 CAD 位姿常量。

## 资产与重建

主文件仅声明参数并组合功能模块：

```text
urdf/
├── dart_system.urdf.xacro
└── modules/
    ├── materials.xacro # 两模式共用材质
    ├── field.xacro          # 场地根坐标与静态地图
    ├── bases.xacro          # 两侧基地、导轨、移动目标
    ├── outposts.xacro       # 两侧前哨站和固定目标
    ├── dart_stations.xacro  # 发射站、舱门、舱底安装基准
    ├── targets.xacro        # 阵营选择、目标参考坐标和场地到镖架安装变换
    ├── launcher.xacro       # 自研镖架、yaw、离轨点
    └── stereo_camera.xacro  # 双目安装和 optical frame
urdf/dart_robot.urdf.xacro   # 实际运行入口：共用参数、launcher、stereo_camera
meshes/
├── shared/                 # 48 份 CAD 定义局部坐标模型，供场地和红蓝双方共用
├── launcher/               # 保留的自研镖架 STL
└── stereo_camera/          # 保留的相机和镜头 STL
```

48 个公共 DAE 和保留的 4 个硬件 STL 均由 Git LFS 管理。
- `metadata/field_alignment_2027.json`：场地坐标基准和装配变换。
- `metadata/rm2027_mechanisms.json`：机构依据、目标面号、每个 visual 的零位显示变换。
- `metadata/mesh_reuse.json`：CAD 零件定义与公共文件、97 个 visual 实例装配变换的对应关系。
- `tools/align_field_2027.py`：从提取缓存重建场地对齐。
- `tools/build_rm2027_description.py`：从拆分缓存生成场景；需 `numpy`、`scipy` 和保留的镖架源 xacro。

```bash
python3 tools/build_rm2027_description.py /path/to/rmuc2027_v1 \
  --launcher-source /path/to/launcher_source.xacro
colcon test --packages-select dart_description
colcon test-result --verbose
```

生成器会同步生成场景入口 `dart_system.urdf.xacro`、运行入口 `dart_robot.urdf.xacro` 和共用模块；修改机构生成规则后应重建并复查。镖架源文件可使用当前模块化入口，生成器会展开相对 include，读取 `dart_base_link` 开始的机构和相机链。
旧场地、旧基地和旧检测模块 STL 已删除，meshes 仅保留当前场景使用的资产。
源 CAD 已有的两个无法网格化面仍保留缺失记录；本模型尚不是封闭碰撞或动力学模型。

## 显示模型优化

`metadata/display_meshes.json` 的 `assets` 保留复用前装配导出的统计，`reuse` 记录当前公共模型的三角面数、体积和检查结果。
处理包括 1 微米顶点合并、按材质边界限制的共面合并、保护锐边的加权减面，随后按 CAD 定义输出公共 DAE，将实例装配变换移到 URDF visual。
绿灯发光面直接保留母版几何；包围尺寸偏差超过 2 mm 的部件退回保守共面合并。
包围尺寸检查不是全表面的误差上界，简化模型只用于显示，不替代标定或碰撞母版。

从 STEP 提取缓存重建（Blender、Python numpy/scipy，以及验证用 libassimp）：

```bash
blender -b -t 6 --python-exit-code 1 --python tools/optimize_display_meshes.py -- /path/to/rmuc2027_v1
blender -b -t 6 --python-exit-code 1 --python tools/shade_display_meshes.py -- /path/to/rmuc2027_v1
python3 tools/export_display_meshes.py /path/to/rmuc2027_v1
python3 tools/validate_display_meshes.py /path/to/rmuc2027_v1 --install
```

输出保存在资产目录 `optimized/`，公共模型在 `optimized/reused/`。验证通过后才替换包内显示资产；安装步骤同步更新 visual 和审计变换并删除已替换的旧装配 DAE。

复用保留不同 CAD 定义的材质差异，不使用统一 URDF 材质覆盖黑色、绿灯或队色。红蓝双方同定义的模型及静态场地重复件引用同一文件；97 个 visual 仍挂在原有 link 下，不增加 TF。验证逐顶点和法线比较公共模型重新装配后的结果，并逐材质核对三角形索引。此次坐标文本舍入差异小于 1 微米，未进一步减面。
法线处理单独输出到 `optimized/shaded/`：20° 以上的棱角保留硬边，缓曲面平滑，逐三角形检查坐标完全不变。
重新减面后必须重新运行法线处理再导出。CAD 缓存保留线性颜色，DAE 导出时转换为默认 RViz 窗口使用的 sRGB 显示值（例如线性黑色 0.0331 对应显示值 0.2），透明度不变。
环境光颜色与漫反射颜色一致，镜面反射为零；RViz 加载时还会将环境光乘以 0.5。若以后切换到启用线性光照及 sRGB 帧缓冲的渲染器，应重新配置颜色输出，避免重复转换。
`build_rm2027_description.py` 会优先使用已有验证记录的 optimized 资产，避免重建 URDF 时覆盖回高精度地图。

前哨站中部三块装甲的中心外表面由 `metadata/display_material_overrides.json` 按 CAD 面号显式修正为黑色；仅在导出显示模型时应用，原始 CAD 缓存不变。导出会检查匹配平面、范围、原颜色和面积，避免将周围金属件一起改黑。
RViz 默认隐藏前哨站的目标模块结构帧及无前缀目标别名，仅显示两侧实际绿灯和装甲测量帧；TF 本身仍保留，几何接口不变。需要调试结构坐标时可在 TF 的 Frames 中勾选。

`target_geometry.Geometry` 保留为场景目标分析工具，默认读取完整场景；实际双目测量使用运行模式的 TF，不依赖此工具或地图中的目标坐标。

## 运行与离线维护目录

- `scripts/`：场景关节状态和双目视线两个 ROS 节点。
- `config/`：查看模式使用的 `sensor_fov.yaml`。
- `target_geometry/`：独立安装的场景目标分析 Python 库。
- `tools/`：CAD 对齐、描述生成、减面、法线、导出、复用和验证工具。
- `metadata/`：CAD 基准、材质修正规则、机构依据及验证记录。

`tools/` 和 `metadata/` 供源码维护与测试使用，不随运行资源安装。离线工具请从包源码目录执行；它们仍读取外部资产目录中的 CAD 缓存。
