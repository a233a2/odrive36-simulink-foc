"""Resolve the project and external Windows tools without starting a build."""
import os
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

from build import ROOT, compiler

EIDE_TOOLS = Path.home() / '.eide/tools'


def is_project(path):
    return (
        (path / 'stm32f405_flash.ld').is_file()
        and (path / 'simulink/build_odrive_foc.m').is_file()
        and (path / 'simulink/firmware/foc_simulink.c').is_file()
    )


def resolve_project(path):
    path = Path(path).expanduser().resolve()
    for candidate in (path, path / 'CM_Odrive'):
        if is_project(candidate):
            return candidate
    raise ValueError('请选择包含 STM32 固件和 Simulink 构建脚本的 CM_Odrive 工程目录。')


def detect_project():
    start = Path(sys.executable).parent if getattr(sys, 'frozen', False) else ROOT
    for base in (start, *start.parents, Path.cwd()):
        for candidate in (base, base / 'CM_Odrive'):
            if is_project(candidate):
                return candidate
    return None


def matlab_release(executable):
    try:
        details = ET.parse(executable.parent.parent / 'VersionInfo.xml')
        return details.findtext('release', '').strip()
    except (OSError, ET.ParseError):
        return ''


def detect_matlab():
    candidates = []
    location = os.environ.get('MATLAB_ROOT')
    if location:
        candidates.append(Path(location) / 'bin/matlab.exe')
    if os.name == 'nt':
        import winreg
        for scope in (winreg.HKEY_LOCAL_MACHINE, winreg.HKEY_CURRENT_USER):
            try:
                with winreg.OpenKey(scope, r'SOFTWARE\MathWorks\MATLAB\R2024b') as key:
                    location, _ = winreg.QueryValueEx(key, 'MATLABROOT')
                candidates.append(Path(location) / 'bin/matlab.exe')
            except OSError:
                pass
        program_files = os.environ.get('ProgramFiles')
        if program_files:
            candidates.append(Path(program_files) / 'MATLAB/R2024b/bin/matlab.exe')
    on_path = shutil.which('matlab')
    if on_path:
        candidates.append(Path(on_path))
    for candidate in candidates:
        if candidate.is_file() and matlab_release(candidate) == 'R2024b':
            return candidate.resolve()
    return None


def validate_matlab(path):
    if not path.is_file():
        raise ValueError('找不到 MATLAB。请选择 R2024b 安装目录下的 bin\\matlab.exe。')
    release = matlab_release(path)
    if release and release != 'R2024b':
        raise ValueError(f'当前 MATLAB 为 {release}。工程配置需要 R2024b。')
    return release


def validate_gcc(path):
    if not path.is_file():
        raise ValueError('找不到 ARM GCC。请选择 arm-none-eabi-gcc.exe。')
    for name in ('arm-none-eabi-objcopy', 'arm-none-eabi-size'):
        companion = path.with_name(name + path.suffix)
        if not companion.is_file():
            raise ValueError(f'ARM GCC 工具链缺少 {companion.name}。请选择完整的 bin 目录。')


def detect_gcc():
    for candidate in (compiler(), EIDE_TOOLS / 'gcc_arm/bin/arm-none-eabi-gcc.exe',
                      Path(shutil.which('arm-none-eabi-gcc') or 'arm-none-eabi-gcc')):
        try:
            validate_gcc(candidate)
        except ValueError:
            continue
        return candidate.resolve()
    return None


def openocd_scripts(path):
    if not path.is_file():
        raise ValueError('找不到 OpenOCD。请选择 openocd.exe。')
    local = path.parent
    for scripts in (local.parent / 'share/openocd/scripts', local.parent / 'scripts', local / 'scripts'):
        if (scripts / 'interface/cmsis-dap.cfg').is_file() and (scripts / 'target/stm32f4x.cfg').is_file():
            return scripts
    raise ValueError('OpenOCD 缺少 CMSIS-DAP 或 STM32F4 配置。请选择完整的安装目录。')


def detect_openocd():
    candidates = sorted(EIDE_TOOLS.glob('openocd*/bin/openocd.exe'))
    on_path = shutil.which('openocd')
    if on_path:
        candidates.append(Path(on_path))
    for candidate in candidates:
        try:
            openocd_scripts(candidate)
        except ValueError:
            continue
        return candidate.resolve()
    return None
