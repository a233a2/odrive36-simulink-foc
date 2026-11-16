"""Run the Cortex-M4 FOC/app regression with stubbed peripherals under Unicorn."""
import argparse
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import sys

from build import ROOT, compiler, simulink_sources


def replay_header(out, fixture):
    data = json.loads(Path(fixture).read_text(encoding='utf-8'))
    if data['count'] != len(data['inputs']) or data['count'] != len(data['outputs']):
        raise ValueError('Simulink replay count does not match its data')
    rows = []
    for inputs, outputs in zip(data['inputs'], data['outputs']):
        if len(inputs) != 15 or len(outputs) != 9 or not all(map(math.isfinite, inputs + outputs)):
            raise ValueError('Invalid shape or number in Simulink replay')
        u = ', '.join(f'{value:#.9g}f' for value in inputs)
        y = ', '.join(f'{value:#.9g}f' for value in outputs)
        rows.append(f'    {{{{{u}}}, {{{y}}}}},')
    out.write_text('/* Simulink normal-mode output for each input tick. */\n'
                   'static const struct { float input[15], output[9]; } foc_replay[] = {\n'
                   + '\n'.join(rows) + '\n};\n', encoding='ascii')


def use_test_deps(path):
    dependencies = str(path)
    if dependencies not in sys.path:
        sys.path.insert(0, dependencies)


def emulate_firmware(elf_path, report=print):
    from elftools.elf.elffile import ELFFile
    from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
    from unicorn.arm_const import (UC_ARM_REG_C1_C0_2, UC_ARM_REG_FPEXC, UC_ARM_REG_SP,
                                   UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC)
    mu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    mu.mem_map(0x08000000, 1024 * 1024)
    mu.mem_map(0x20000000, 128 * 1024)
    mu.reg_write(UC_ARM_REG_C1_C0_2, 0xf00000)
    mu.reg_write(UC_ARM_REG_FPEXC, 0x40000000)
    with elf_path.open('rb') as stream:
        elf = ELFFile(stream)
        symbols = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
        for segment in elf.iter_segments():
            if segment['p_type'] == 'PT_LOAD' and segment['p_filesz']:
                mu.mem_write(segment['p_vaddr'], segment.data())
    mu.reg_write(UC_ARM_REG_SP, 0x2001fff0)
    mu.reg_write(UC_ARM_REG_LR, 0x080ff001)
    try:
        mu.emu_start(symbols['run_tests'] | 1, 0x080ff000, timeout=55000000, count=1000000000)
    except Exception:
        report('Emulation PC=' + hex(mu.reg_read(UC_ARM_REG_PC)))
        raise
    checks = struct.unpack('<I', mu.mem_read(symbols['test_checks'], 4))[0]
    failure = struct.unpack('<I', mu.mem_read(symbols['test_failure'], 4))[0]
    result = mu.reg_read(UC_ARM_REG_R0)
    pc = mu.reg_read(UC_ARM_REG_PC)
    return {'checks': checks, 'failure_line': failure, 'return_code': result,
            'pc': hex(pc), 'passed': not failure and result == 0 and pc == 0x080ff000}


def exercise_arm_runtime():
    from elftools.elf.elffile import ELFFile
    from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
    from unicorn.arm_const import UC_ARM_REG_R0

    address = 0x08000000
    mu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    mu.mem_map(address, 4096)
    mu.mem_write(address, b'\x07\x20')  # Thumb MOVS r0, #7
    mu.emu_start(address | 1, address + 2, count=1)
    if mu.reg_read(UC_ARM_REG_R0) != 7:
        raise RuntimeError('ARM 仿真自检指令执行失败。')


def run_emulator_process(arguments, *, cwd, run, report):
    # Unicorn uses non-local jumps that conflict with Windows CFG. Its frozen
    # worker keeps those jumps and native failures outside the UI process.
    worker = Path(sys._MEIPASS) / 'arm/ODriveFOC-ARM.exe'
    env = dict(os.environ, PYINSTALLER_RESET_ENVIRONMENT='1')
    child = run([str(worker), *arguments], cwd=cwd, env=env,
                capture_output=True, text=True, timeout=90)
    if child.returncode:
        report(child.stdout)
        raise RuntimeError(f'ARM 仿真进程退出状态 0x{child.returncode & 0xFFFFFFFF:08X}。')
    return json.loads(child.stdout)


def check_arm_runtime(*, run=subprocess.run, report=print):
    if getattr(sys, 'frozen', False):
        result = run_emulator_process(['--check-runtime'], cwd=Path.cwd(), run=run, report=report)
        if result.get('passed') is not True:
            raise RuntimeError('ARM 仿真进程自检失败。')
    else:
        use_test_deps(ROOT / 'tmp/testdeps')
        exercise_arm_runtime()


def run_tests(out, generated=None, fixture=None, *, project=ROOT, gcc=None,
              run=subprocess.run, report=print, test_deps=None):
    project = Path(project).resolve()
    out = Path(out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    elf_path = out / 'firmware_tests.elf'
    sources, flags = simulink_sources(generated)
    if generated is not None:
        replay_header(out / 'foc_replay.h', fixture)
        flags += ['-I' + str(out)]
    cmd = [str(gcc if gcc is not None else compiler()), '-mcpu=cortex-m4', '-mthumb', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard',
           '-O2', '-g', '-fno-math-errno', '-DSTM32F405xx', '-Iinclude', '-Ivendor/Device', '-Ivendor/CMSIS',
           '-ffunction-sections', '-fdata-sections'] + flags + [
           'src/foc.c', 'src/app.c', 'src/syscalls.c', 'tests/firmware_harness.c'] + sources + [
           '-nostartfiles', '-Tstm32f405_flash.ld', '-Wl,--gc-sections', '-Wl,-u,run_tests',
           '--specs=nano.specs', '--specs=nosys.specs', '-u', '_printf_float', '-u', '_scanf_float',
           '-lm', '-lc', '-o', str(elf_path)]
    run(cmd, cwd=project, check=True)
    if getattr(sys, 'frozen', False):
        result = run_emulator_process(['--replay', str(elf_path)], cwd=out, run=run, report=report)
    else:
        use_test_deps(test_deps if test_deps is not None else project / 'tmp/testdeps')
        result = emulate_firmware(elf_path, report)
    result.update(backend='Simulink C' if generated is not None else 'native C',
                  scope='ARM FOC and application logic with stubbed peripheral drivers')
    (out / 'results.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    report(json.dumps(result, indent=2))
    if not result['passed']:
        raise RuntimeError('ARM regression failed at line ' + str(failure))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--check-runtime', action='store_true', help=argparse.SUPPRESS)
    action.add_argument('--replay', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.replay or args.check_runtime:
        if not getattr(sys, 'frozen', False):
            use_test_deps(ROOT / 'tmp/testdeps')
        if args.check_runtime:
            exercise_arm_runtime()
            print(json.dumps({'passed': True}), flush=True)
        else:
            print(json.dumps(emulate_firmware(args.replay)), flush=True)
    else:
        run_tests(ROOT / 'build/tests')
