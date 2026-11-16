"""Reproducible GCC build using EIDE's installed compiler; no IDE required for CI."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
PROJECT_EXPORT = Path('simulink/deployed/current')
EIDE_FOC_TARGET = 'Simulink_FOC'
SOURCES = ['src/main.c', 'src/board.c', 'src/drv8301.c', 'src/encoder.c', 'src/foc.c',
           'src/app.c', 'src/usb_console.c', 'src/syscalls.c', 'startup/startup_stm32f405xx.s',
           'vendor/tinyusb/src/tusb.c', 'vendor/tinyusb/src/common/tusb_fifo.c',
           'vendor/tinyusb/src/device/usbd.c', 'vendor/tinyusb/src/device/usbd_control.c',
           'vendor/tinyusb/src/class/cdc/cdc_device.c',
           'vendor/tinyusb/src/portable/synopsys/dwc2/dcd_dwc2.c']
INCLUDES = ['include', 'vendor/Device', 'vendor/CMSIS', 'vendor/tinyusb/src']

def compiler():
    candidate = os.environ.get('ARM_GCC_BIN')
    if candidate:
        return Path(candidate) / 'arm-none-eabi-gcc.exe'
    installed = Path.home() / '.eide/tools/gcc_arm/bin/arm-none-eabi-gcc.exe'
    if installed.exists():
        return installed
    return Path(shutil.which('arm-none-eabi-gcc') or 'arm-none-eabi-gcc')

def controller_files(generated):
    """Select the headers and target sources from the flat Simulink export."""
    generated = Path(generated).resolve()
    for name in ('odrive_foc_core.c', 'odrive_foc_core.h', 'odrive_foc_contract.h'):
        if not (generated / name).is_file():
            raise FileNotFoundError('Simulink export is missing ' + name)
    # The ADC IRQ and Cortex-M startup provide the scheduler and entry point.
    return [path for path in sorted(generated.iterdir())
            if path.is_file() and path.suffix.lower() in ('.c', '.h')
            and path.name.lower() not in ('rt_main.c', 'ert_main.c')]


def simulink_sources(generated):
    """Compile the board adapter and the flat C export from Simulink Coder."""
    if generated is None:
        return [], []
    generated = Path(generated).resolve()
    sources = ['simulink/firmware/foc_simulink.c']
    sources += [str(path) for path in controller_files(generated) if path.suffix.lower() == '.c']
    return sources, ['-DCM_USE_SIMULINK_FOC', '-I' + str(generated)]


def build_firmware(out, generated=None, *, project=ROOT, gcc=None,
                   run=subprocess.run, report=print):
    project = Path(project).resolve()
    out = Path(out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    gcc = Path(gcc) if gcc is not None else compiler()
    stem = 'CM_Odrive' if generated is None else 'CM_Odrive_Simulink'
    extra_sources, extra_flags = simulink_sources(generated)
    flags = ['-mcpu=cortex-m4', '-mthumb', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard',
             '-O2', '-g3', '-ffunction-sections', '-fdata-sections', '-fno-common', '-fno-math-errno',
             '-Wall', '-Wextra', '-Wdouble-promotion', '-DSTM32F405xx', '-DHSE_VALUE=8000000',
             '-DUSE_FULL_ASSERT=0'] + ['-I' + p for p in INCLUDES] + extra_flags
    objects, commands = [], []
    for source in SOURCES + extra_sources:
        path = Path(source)
        obj_name = path.name if path.is_absolute() else source.replace('/', '_')
        obj = out / (obj_name + '.o')
        cmd = [str(gcc)] + flags + ['-std=gnu11'] if source.endswith('.c') else [str(gcc)] + flags
        cmd += ['-c', source, '-o', str(obj), '-MMD', '-MP']
        report('CC ' + (path.name if path.is_absolute() else source))
        run(cmd, cwd=project, check=True)
        objects.append(str(obj))
        commands.append({'directory': str(project), 'file': str(project / source), 'arguments': cmd})
    elf = out / (stem + '.elf')
    cmd = [str(gcc)] + flags[:4] + objects + ['-Tstm32f405_flash.ld', '-Wl,--gc-sections',
          '-Wl,-Map=' + str(out / (stem + '.map')), '-Wl,--print-memory-usage',
          '--specs=nano.specs', '--specs=nosys.specs', '-u', '_printf_float', '-u', '_scanf_float',
          '-Wl,--start-group', '-lc', '-lm', '-Wl,--end-group', '-o', str(elf)]
    run(cmd, cwd=project, check=True)
    exe = '.exe' if gcc.suffix == '.exe' else ''
    for kind in ['binary', 'ihex']:
        dest = out / (stem + ('.bin' if kind == 'binary' else '.hex'))
        run([str(gcc.parent / ('arm-none-eabi-objcopy' + exe)), '-O', kind, str(elf), str(dest)], check=True)
    run([str(gcc.parent / ('arm-none-eabi-size' + exe)), str(elf)], check=True)
    db = project if generated is None else out
    (db / 'compile_commands.json').write_text(json.dumps(commands, indent=2))
    report('BUILD OK: ' + str(elf))
    return elf


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--clean', action='store_true')
    parser.add_argument('--simulink', action='store_true', help='build the controller installed in the project')
    args = parser.parse_args()
    out = ROOT / ('build/Simulink_GCC' if args.simulink else 'build/Debug')
    if args.clean and out.exists():
        # Only this script's verified build directory can be removed.
        assert out.resolve().is_relative_to(ROOT.resolve()) if hasattr(Path, 'is_relative_to') else out.parent.parent == ROOT
        shutil.rmtree(str(out))
    build_firmware(out, ROOT / PROJECT_EXPORT if args.simulink else None)

if __name__ == '__main__':
    main()
