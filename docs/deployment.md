# Build and deploy

The Windows desktop tool and command-line entry both use `simulink/build_odrive_foc.m` for code generation and the same ARM GCC build. The supplied model runs in R2024b. Code generation uses Simulink Coder and Embedded Coder with the products listed in the [quick start](../README.md#quick-start).

## Build from a saved model

1. Open `simulink/odrive_foc.slx` and run the simulation to inspect current tracking or change the load step. Save your controller edits.
2. Open `python tools/deploy_gui.py` and select the project and the saved model. Set MATLAB R2024b and `arm-none-eabi-gcc.exe` in the toolchain dialog if the tool needs help finding them. The CLI also accepts `--matlab` and `--gcc`.
3. Choose **移植并构建** and click **一键移植并构建**. The tool validates and exports only `FOC Current Loop`, compiles the board adapter and firmware, and executes the ARM replay when enabled. The default operation runs on the PC.

The following PowerShell commands run the same deployment. For nonstandard installations, set `MATLAB_ROOT` to the MATLAB installation root and `ARM_GCC_BIN` to the Arm GNU Toolchain `bin` directory.

```powershell
python tools/install_test_deps.py
python tools/deploy_foc.py --install-project --verify-arm
```

The build writes a model snapshot, ERT export, ELF, HEX, BIN, and `manifest.json` to `build/Simulink/<run>/`. After compilation and the requested checks pass, the tool verifies the generated source hashes and publishes them to `simulink/deployed/current/`. A failed code generation, compile or replay leaves the previously installed sources in place.

Open `CM_Odrive.code-workspace`, then choose EIDE's `Simulink_FOC` target to compile the installed export. Its output is in `build/Simulink_FOC/`. EIDE's `Debug` target builds the handwritten controller. Use `python tools/build.py --simulink` to rebuild the installed C without launching MATLAB. The default `python tools/build.py` builds the handwritten controller.

### Reproduce the demo plot

Run the demo from the repository root in the MATLAB command window.

```matlab
addpath('simulink')
verify_odrive_motor_demo(fullfile(pwd, 'simulink', 'odrive_foc.slx'), fullfile(pwd, 'build', 'demo'))
```

The demo applies a 0.12 A q-axis command and a 1.4 mN·m opposing load step. The curves in the README are from the averaged inverter and PMSM simulation. The ARM harness uses stub peripherals to test the application and compare the generated controller against Simulink vectors.

## Package the Windows EXE

Use Windows x64 Python 3.12 with Tk and the Visual Studio x64 C++ tools. The GUI uses PyInstaller's standard Windows bootloader. Its internal Unicorn worker uses the `--no-cfg` build to run the ARM emulator in a separate process. [PyInstaller build notes](https://pyinstaller.org/en/stable/common-issues-and-pitfalls.html#windows-executables-and-control-flow-guard-cfg)

```powershell
python tools/install_test_deps.py
python -m venv build/package_gui/venv
python -m venv build/package_gui/replay_venv
$guiPython = 'build/package_gui/venv/Scripts/python.exe'
$replayPython = 'build/package_gui/replay_venv/Scripts/python.exe'
& $guiPython -m pip install PyInstaller==6.22.3
$env:PYINSTALLER_COMPILE_BOOTLOADER = '1'
$env:PYINSTALLER_BOOTLOADER_WAF_ARGS = '--no-cfg'
try {
    & $replayPython -m pip install --no-cache-dir --no-binary PyInstaller PyInstaller==6.22.3
} finally {
    Remove-Item Env:PYINSTALLER_COMPILE_BOOTLOADER
    Remove-Item Env:PYINSTALLER_BOOTLOADER_WAF_ARGS
}
& $guiPython tools/package_deploy_gui.py --replay-python $replayPython
```

The packaging script tests desktop startup and runs a Thumb instruction before publishing `build/Tools/ODriveFOC.exe`. The EXE calls the installed MATLAB and ARM toolchain.

## Development preview scope

The public repository ships the editable model, adapter, board firmware, deployment and packaging sources. Generate the ERT code and firmware locally under your applicable MATLAB license. [MathWorks deployment rights](https://www.mathworks.com/help/pdf_doc/offering/offering.pdf)

The default build mode is offline. The optional DAPLink HID/SWD operation uses OpenOCD and verifies the write. The reference implementation was exercised on Windows 11 x64 for code generation, building and ARM replay. Hardware timing and closed-loop commissioning remain in progress.
