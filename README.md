# OneRobotics A1 for Gazebo

这是 OneRobotics A1 的 Gazebo Harmonic（Gazebo Sim 8）资产仓库，包含右臂、
左臂和双臂站架三个独立模型。模型已从公开 URDF 转换为 Gazebo 原生
SDFormat 1.11；本仓库不修改 Gazebo 仿真器本身。

## 第一次安装

在终端依次运行：

```bash
cd "$HOME/桌面/onerobot_h1_gazebo"
env -u PYTHONPATH uv sync --project gazebo --locked
bash gazebo/scripts/install_harmonic_conda.sh
```

安装过程不需要替换系统自带的 Gazebo Classic。

## 打开模型

以后从任意目录运行这一条命令：

```bash
bash "$HOME/桌面/onerobot_h1_gazebo/gazebo/scripts/open_demo.sh"
```

启动菜单中输入：

- `1`：右臂
- `2`：左臂
- `3`：双臂站架
- `0`：退出

关闭 Gazebo 窗口后会返回菜单。看到完整模型表示本地加载和渲染成功。

如果出现 `Failed to create an OpenGL context`，先运行 `nvidia-smi` 检查显卡
驱动；驱动升级后版本不一致通常需要完整重启电脑。

## 自动验证

```bash
cd "$HOME/桌面/onerobot_h1_gazebo"
env -u PYTHONPATH uv run --project gazebo --locked pytest gazebo/tests -q
```

更完整的本地运动 smoke test 命令见
[Gazebo 详细使用教程](gazebo/README.md)。

## Gazebo Fuel

三个模型已分别发布到 [Gazebo Fuel 模型页](https://app.gazebosim.org/fuel/models)，
Owner 为 `OneRobotics`：

- `OneRobotics A1 Right Arm`
- `OneRobotics A1 Left Arm`
- `OneRobotics A1 Bimanual Stand`

发布操作、清单和复验记录见
[Fuel 发布记录与维护说明](gazebo/PUBLISHING.md)。

## 来源、修改和许可证

- 来源：[`katazen/onerobot_h1`](https://github.com/katazen/onerobot_h1)
- 固定来源提交：`ecf530911284ba0e559f7a24dc222fd8e60d31ed`
- 修改：将公开 URDF 转换为 SDFormat 1.11，并整理 Fuel 所需的
  `model.sdf`、`model.config`、缩略图和元数据；不修改 STL 网格字节。
- A1 机器人资产：OneRobotics © 2026，采用 CC BY 4.0。
- 本仓库新增代码：BSD 3-Clause。

完整许可证及第三方说明见 [`LICENSES/`](LICENSES/)、
[`ASSET_LICENSE_STATUS.md`](ASSET_LICENSE_STATUS.md) 和
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md)。

## 已知限制

- 模型不含夹爪、传感器或 ROS 2 control。
- 碰撞模型保留公开来源的高精度网格，不适合追求高速的大规模接触仿真。
- 右臂、左臂和双臂站架是三个独立 CAD 版本，不应拼接成同一个模型。

仓库中保留的 `source/` 文件用于锁定公开来源和重复生成 Gazebo 资产；普通用户
只需使用 `gazebo/` 目录，不需要运行 Isaac Lab 或 RL 训练。
