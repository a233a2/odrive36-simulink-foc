# ODrive 3.6 Simulink FOC

[![Cortex-M4 固件构建](https://github.com/a233a2/odrive36-simulink-foc/actions/workflows/native-firmware.yml/badge.svg)](https://github.com/a233a2/odrive36-simulink-foc/actions/workflows/native-firmware.yml)
![MATLAB R2024b](https://img.shields.io/badge/MATLAB-R2024b-0076A8?style=flat-square)
![STM32F405](https://img.shields.io/badge/Target-STM32F405-176C60?style=flat-square)
[![MIT](https://img.shields.io/badge/License-MIT-293B52?style=flat-square)](LICENSE)

**把 Simulink 电流环变成 ODrive 3.6 上可编译的 FOC 控制程序。** 上位机一键提取子系统、生成 C、编译 STM32 固件，并把通过验证的源码同步到 EIDE。

[English](README.md) · [接口契约](docs/interfaces.md) · [部署指南](docs/deployment.md) · [版本发布](https://github.com/a233a2/odrive36-simulink-foc/releases)

![FOC 电流环、平均逆变器、带负载输入的 PMSM 和三个示波器](docs/media/simulink-model.png)

## 接口与部署

```mermaid
flowchart LR
    A["保存 SLX"] --> B["提取 FOC 电流环"]
    B --> C["ERT 生成 C 和头文件"]
    C --> D["适配层与 ARM GCC"]
    D --> E["固件 ELF / HEX / BIN"]
    D --> F["同步源码到 EIDE"]
```

板级固件以 **20 kHz** 通过适配层传入相电流、电角度和各轴状态，调用 `odrive_foc_core_step()` 后检查占空比并更新 PWM。编码器、速度和位置外环、故障关断由板级固件执行。

## 快速开始

准备 Windows x64、带 Tk 的 Python 3.12、Arm GNU Toolchain，以及已授权的 MATLAB R2024b、Simulink、MATLAB Coder、Simulink Coder 和 Embedded Coder。

```powershell
git clone https://github.com/a233a2/odrive36-simulink-foc.git
cd odrive36-simulink-foc
python tools/install_test_deps.py
python tools/deploy_gui.py
```

打开 `simulink/odrive_foc.slx`，点 **Run** 查看电流阶跃和负载扰动。保存控制器改动后，在上位机点 **一键移植并构建**。生成源码同步到 `simulink/deployed/current/`。用 EIDE 打开 `CM_Odrive.code-workspace`，选择 `Simulink_FOC` 可以直接重编。

<details>
<summary>命令行操作</summary>

```powershell
python tools/deploy_foc.py --install-project --verify-arm
python tools/build.py --simulink
```

每次部署的固件、日志和清单位于 `build/Simulink/<run>/`。工具链设置和 Windows EXE 打包见[部署指南](docs/deployment.md)。

</details>

<table>
  <tr>
    <td width="50%"><a href="docs/media/deploy-tool.png"><img src="docs/media/deploy-tool.png" alt="一键部署上位机"></a></td>
    <td width="50%"><a href="docs/media/load-step.png"><img src="docs/media/load-step.png" alt="仿真电流、转速和负载转矩波形"></a></td>
  </tr>
  <tr><td>上位机部署界面</td><td>PMSM 电流跟踪与负载阶跃仿真</td></tr>
</table>

首发为开发预览。**96 组 Simulink 向量、3,090 项 ARM 检查**通过，EIDE 的 `Simulink_FOC` 和 `Debug` 均已重编通过。实机时序与闭环运行仍在验证。原创模型和源码采用 [MIT](LICENSE)，第三方组件沿用[原许可证](THIRD_PARTY_NOTICES.md)。
