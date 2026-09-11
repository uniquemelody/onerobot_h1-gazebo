repository（仓库）就是带版本记录的项目文件夹；代码 1 在这里保存 A1 源资产和厂商侧集成。
URDF 是机器人的原始结构描述；本项目把锁定的公开 URDF 当作输入。
SDF 是 Gazebo 原生模型格式；本项目将输入转换成三个可本地验证的 SDF 1.11 模型。
Gazebo 是电脑上加载、显示和运行模型的仿真器；这里固定使用 Gazebo Harmonic / Sim 8。
Fuel 是 Gazebo 的在线模型仓库；本地查看或打包都不代表已发布。

# A1 Gazebo 零基础本地查看

这页只讲“在本机看模型”。它不要求 Fuel 帐号、令牌或发布权限。组织授权维护者如需审批和发布，请转到完整的 [PUBLISHING.md](PUBLISHING.md)。

## 先分清三个代码位置

- 代码 1：[`katazen/onerobot_h1`](https://github.com/katazen/onerobot_h1.git) 保存 A1 source assets 和 vendor-side integrations。锁定的公开输入提交是 `ecf530911284ba0e559f7a24dc222fd8e60d31ed`。
- 代码 2：`T1Amoo/IsaacLab` 是 IsaacLab 官方上游仓库的个人 fork；其中的 `contrib/onerobotics-a1-reach` branch 是 IsaacLab-repository contribution，不是代码 1 的 branch。
- 本 Gazebo 工作：代码 1 的本地 `feat/gazebo-harmonic-fuel-assets` feature branch；它未 push，不是 Gazebo / gz-sim 引擎 fork。Gazebo implementation is not on the public origin，所以新克隆的 GitHub 仓库目前没有这条本地分支或其中的 Gazebo 集成。

## 一次性准备

在本仓库根目录运行一次。它会同步本地 Python 工具、检查锁定来源，并安装隔离的 Harmonic 环境；不需要 sudo，也不会替换系统 Gazebo Classic。

~~~bash
set -euo pipefail
cd "$HOME/桌面/onerobot_h1-gazebo"
env -u PYTHONPATH uv sync --project gazebo --locked
env -u PYTHONPATH uv run --project gazebo python -m onerobotics_a1_gazebo.source_lock --check gazebo/generated-manifest.json
/usr/bin/gazebo --version
bash gazebo/scripts/install_harmonic_conda.sh
/usr/bin/gazebo --version
~~~

来源检查应以 `SOURCE_LOCK_OK` 结束并报告 39 files。不要修改 `source/h1_reach/h1_reach/assets/urdf/A1_2026` 中的锁定源文件。

## 日常查看：只用这一条启动路径

下面这一条命令可从任何目录运行。启动器会自行导出并验证本地模型、选择 Sim 8、只解析本地 `GZ_SIM_RESOURCE_PATH`，并隔离 HOME/XDG 状态；它只保留当前图形会话所需的 `DISPLAY` 或 `WAYLAND_DISPLAY` 等变量。

~~~bash
set -euo pipefail
bash "$HOME/桌面/onerobot_h1-gazebo/gazebo/scripts/open_demo.sh"
~~~

菜单含义：

- `1. 右臂`：打开右臂演示。
- `2. 左臂`：打开左臂演示。
- `3. 双臂站架`：打开独立的双臂站架演示。
- `0. 退出`：退出启动器；关闭 Gazebo 窗口后会回到这个菜单。

视觉核对：模型应完整、没有明显裁切或空白；方向和光照应清楚；右臂、左臂、双臂站架应分别作为三个独立模型出现。right arm、left arm 和 bimanual stand 是 separate CAD revisions，must not be assembled together。

GUI success proves local load/render only（仅证明本地 load/render）：它不证明每项物理性质，也不证明模型已经上传、发布或能被其他新 clone 获取。

## 单独运行四个运动 smoke cases

这不是日常查看命令；它会对三个模型运行四个受限的真实关节运动检查。

~~~bash
set -euo pipefail
cd "$HOME/桌面/onerobot_h1-gazebo"
env -u PYTHONPATH uv run --project gazebo python -m onerobotics_a1_gazebo.package --output dist/gazebo-fuel
env -u PYTHONPATH uv run --project gazebo python -m onerobotics_a1_gazebo.validate dist/gazebo-fuel
bash gazebo/scripts/check_harmonic.sh dist/gazebo-fuel
env -u PYTHONPATH uv run --project gazebo python -m onerobotics_a1_gazebo.demo generate --models dist/gazebo-fuel --output gazebo/worlds
bash gazebo/scripts/run_smoke_tests.sh dist/gazebo-fuel gazebo/worlds
~~~

运行完成会先给出 four SMOKE_TEST_VALID（右臂、左臂，以及双臂站架的 `joint_r1` 和 `joint_l1`），最后成功标记是 `SMOKE_TEST_OK: 4 cases across 3 models`。这组命令仅用于独立 smoke；不要通过手工设置 Gazebo 环境来替代日常查看启动器。

## 当前限制与状态

- high-poly collision meshes：优先保留公开源几何，不适合高性能接触仿真。
- no gripper。
- no ROS 2 control。
- no sensors。
- no invented dynamics or friction：没有来源的数据不会凭空补阻尼、摩擦或控制增益。

当前 Gazebo 分支仅在本机且尚未 push；新 clone 没有这条 branch。需要代表组织审核缩略图、准备 manifest、交接安全客户端或执行 Fuel 后续复验的授权维护者，请阅读 [PUBLISHING.md](PUBLISHING.md)。
