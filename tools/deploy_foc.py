"""Export the FOC subsystem in MATLAB R2024b and build the ODrive 3.6 firmware."""
import argparse
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime
import hashlib
import json
import locale
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
from typing import Optional

from build import PROJECT_EXPORT, ROOT, build_firmware, compiler, controller_files
from deploy_environment import (detect_gcc, detect_matlab, detect_openocd,
                                openocd_scripts, resolve_project, validate_gcc, validate_matlab)
from install_foc import has_eide_target, publish_controller


@dataclass
class Deployment:
    project: Path
    model: Path
    matlab: Path
    gcc: Path
    output: Path
    verify_arm: bool = False
    create_model: bool = False
    flash: bool = False
    openocd: Optional[Path] = None
    install_project: bool = False

    def validate(self):
        validate_matlab(self.matlab)
        validate_gcc(self.gcc)
        if self.model.name.lower() != 'odrive_foc.slx' or not self.model.parent.is_dir():
            raise ValueError('请选择现有目录中的 odrive_foc.slx。')
        if self.create_model and self.model.exists():
            raise ValueError('模型已存在。新建模型时请选择其他目录。')
        if not self.create_model and not self.model.is_file():
            raise ValueError('找不到 odrive_foc.slx。请选择保存好的模型。')
        if self.install_project and not has_eide_target(self.project):
            raise ValueError('工程缺少 EIDE 的 Simulink_FOC 目标。更新工程后即可一键移植。')
        source_root = self.project / PROJECT_EXPORT.parent
        if self.output == source_root or source_root in self.output.parents:
            raise ValueError('请把构建产物保存在生成源码目录以外，例如 build/Simulink。')
        if self.flash:
            if self.openocd is None:
                raise ValueError('烧录需要 OpenOCD。请在工具链设置中选择 openocd.exe。')
            openocd_scripts(self.openocd)


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def generate(options, project, stage, run, report):
    # MATLAB works from a per-build copy so a saved edit cannot alter an active build.
    snapshot = stage / 'odrive_foc.slx'
    if not options.create_model:
        shutil.copyfile(options.model, snapshot)
    env = dict(os.environ, ODRIVE_FOC_MATLAB_SOURCES=str(project / 'simulink'),
               ODRIVE_FOC_MODEL_SNAPSHOT=str(snapshot), ODRIVE_FOC_BUILD=str(stage))
    statement = "addpath(getenv('ODRIVE_FOC_MATLAB_SOURCES')); "
    if options.create_model:
        statement += "create_odrive_foc_model(getenv('ODRIVE_FOC_BUILD')); "
    statement += "build_odrive_foc(getenv('ODRIVE_FOC_MODEL_SNAPSHOT'),getenv('ODRIVE_FOC_BUILD'));"
    log = stage / 'matlab.log'
    command = [str(options.matlab), '-wait', '-logfile', str(log), '-batch', statement]
    report('MATLAB R2024b: ' + str(log))
    result = run(command, cwd=stage, env=env, check=False)
    if result.returncode:
        if log.is_file():
            raw = log.read_bytes()
            try:
                diagnostics = raw.decode('utf-8')
            except UnicodeDecodeError:
                diagnostics = raw.decode(locale.getpreferredencoding(False), errors='replace')
            report(diagnostics[-8000:])
        detail = ''
        if not log.is_file() or not log.stat().st_size:
            detail = ' 请打开 R2024b 检查登录与许可证，然后重试。'
        raise RuntimeError(f'MATLAB 退出状态 {result.returncode}。{detail}')
    if options.create_model:
        # Only the explicit creation mode writes back a model. Exclusive create
        # prevents overwriting an edit saved by another MATLAB session.
        with options.model.open('xb') as destination:
            destination.write(snapshot.read_bytes())
    return snapshot


def flash_firmware(openocd, elf, run=subprocess.run):
    scripts = openocd_scripts(openocd)
    command = [str(openocd), '-s', str(scripts), '-f', 'interface/cmsis-dap.cfg',
               '-c', 'if {[llength [info commands cmsis_dap_backend]]} {'
                     'cmsis_dap_backend hid; cmsis_dap_vid_pid 0xc251 0xf001'
                     '} else {cmsis-dap backend hid; cmsis-dap vid_pid 0xc251 0xf001}; '
                     'transport select swd',
               '-f', 'target/stm32f4x.cfg',
               '-c', 'gdb_port disabled; tcl_port disabled; telnet_port disabled; adapter speed 100; '
                     'stm32f4x.cpu configure -event reset-start {adapter speed 100}; '
                     'stm32f4x.cpu configure -event reset-init {}',
               '-c', 'init; set id [lindex [read_memory 0xE0042000 32 1] 0]; '
                     'if {($id & 0xFFF) != 0x413} {error {Expected STM32F405/F407 family}}; '
                     'program {' + elf.name + '} verify reset exit']
    # The ELF is linked at 0x08000000. OpenOCD writes the image sectors and
    # verifies them before resetting the controller.
    run(command, cwd=elf.parent, check=True, timeout=180)


def simple_build_path(path):
    # Vendor makefiles can impose additional restrictions on Windows paths.
    return len(str(path)) <= 120 and re.fullmatch(r'[A-Za-z0-9_./\\:-]+', str(path)) is not None


@contextmanager
def working_directories(project, stage, report):
    if os.name != 'nt' or (simple_build_path(project) and simple_build_path(stage)):
        yield project, stage
        return
    # A short ASCII work directory keeps MATLAB and GCC independent of the
    # selected model/result path. Only published outputs go back to the user.
    possible_parents = (Path(tempfile.gettempdir()), Path(os.environ.get('PUBLIC', tempfile.gettempdir())))
    scratch_parent = next((p for p in possible_parents if simple_build_path(p) and p.is_dir()), None)
    if scratch_parent is None:
        raise ValueError('Windows 工具链需要短路径作为工作目录。请选择较短的工程和产物路径。')
    with tempfile.TemporaryDirectory(prefix='odfoc_', dir=scratch_parent) as directory:
        scratch = Path(directory)
        work = scratch / 'out'
        work.mkdir()
        if not simple_build_path(project):
            source = scratch / 'project'
            source.mkdir()
            for name in ('src', 'include', 'startup', 'vendor', 'simulink', 'tests'):
                if (project / name).is_dir():
                    shutil.copytree(project / name, source / name, ignore=shutil.ignore_patterns(
                        'slprj', '*.slxc', '*.autosave', 'deployed', '.foc-*'))
            shutil.copyfile(project / 'stm32f405_flash.ld', source / 'stm32f405_flash.ld')
            project = source
        report('工具链工作目录: ' + str(work))
        try:
            yield project, work
        finally:
            # The manifest is saved after these files are published.
            for name in ('odrive_foc.slx', 'odrive_foc_core.slx', 'odrive_foc_c.zip',
                         'matlab_environment.json', 'matlab.log', 'simulation.json'):
                current = work / name
                if current.is_file():
                    shutil.copy2(current, stage / name)
            for name in ('firmware', 'generated', 'tests'):
                current = work / name
                if current.is_dir():
                    shutil.copytree(current, stage / name)


def deploy(options, *, report=print, run=subprocess.run, on_phase=None):
    options.validate()
    options.output.mkdir(parents=True, exist_ok=True)
    now = datetime.now().astimezone()
    stage = Path(tempfile.mkdtemp(prefix=now.strftime('%Y%m%d-%H%M%S-'), dir=options.output))
    def phase(name):
        if on_phase is not None:
            on_phase(name, stage)

    phase('model')
    report('Build folder: ' + str(stage))
    with working_directories(options.project, stage, report) as (project, work):
        snapshot = generate(options, project, work, run, report)
        generated = work / 'generated'
        source_hashes = {path.name: sha256(path) for path in controller_files(generated)}
        phase('compile')
        elf = build_firmware(work / 'firmware', generated, project=project, gcc=options.gcc,
                             run=run, report=report)
        arm = None
        if options.verify_arm:
            phase('verify')
            from test_firmware import run_tests
            arm = run_tests(work / 'tests', generated, work / 'simulation.json', project=project,
                            gcc=options.gcc, run=run, report=report,
                            test_deps=options.project / 'tmp/testdeps')
        files = [snapshot, work / 'odrive_foc_core.slx', work / 'odrive_foc_c.zip',
                 work / 'matlab_environment.json', elf, elf.with_suffix('.hex'), elf.with_suffix('.bin')]
        version = run([str(options.gcc), '-dumpfullversion'], check=True,
                      capture_output=True, text=True).stdout.strip()
        manifest = {
            'created_at': now.isoformat(), 'target': 'STM32F405RGT6',
            'load_address': '0x08000000', 'model': snapshot.name,
            'controller_subsystem': 'odrive_foc/FOC Motion Controller',
            'codegen_model': 'odrive_foc_core.slx', 'compiler': version,
            'simulation': 'simulation.json', 'arm_regression': arm,
            'generated_sources': source_hashes,
            'artifacts': {path.relative_to(work).as_posix(): sha256(path) for path in files},
        }
        if options.install_project:
            phase('install')
            installed = publish_controller(options.project, generated, stage, manifest)
            manifest['project_install'] = installed
            report('EIDE Simulink_FOC 已同步到 ' + str(options.project / installed['directory']))
        if options.flash:
            phase('flash')
            try:
                flash_firmware(options.openocd, elf, run)
            except (OSError, ValueError, subprocess.SubprocessError) as error:
                # Preserve the verified firmware as a useful result of this run.
                flash_error = error
            else:
                flash_error = None
        else:
            flash_error = None

    pending = stage / 'manifest.tmp'
    pending.write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    pending.replace(stage / 'manifest.json')
    report('Firmware: ' + str(stage / 'firmware' / elf.name))
    if flash_error is not None:
        raise RuntimeError(f'固件已生成。烧录失败：{flash_error}') from flash_error
    return stage


def cli_report(message):
    encoding = sys.stdout.encoding or 'utf-8'
    print(str(message).encode(encoding, 'backslashreplace').decode(encoding), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, default=ROOT, help='CM_Odrive project folder')
    parser.add_argument('--matlab', type=Path, help='R2024b matlab.exe')
    parser.add_argument('--gcc', type=Path, help='arm-none-eabi-gcc executable')
    parser.add_argument('--model', type=Path)
    parser.add_argument('--output', type=Path, help='parent folder for build results')
    parser.add_argument('--create-model', action='store_true', help='create a new .slx from the block sources')
    parser.add_argument('--verify-arm', action='store_true', help='replay Simulink vectors in the ARM regression')
    parser.add_argument('--install-project', action='store_true', help='publish the built C to the EIDE Simulink_FOC target')
    parser.add_argument('--flash', action='store_true', help='write and verify via the connected DAPLink after building')
    parser.add_argument('--openocd', type=Path, help='OpenOCD executable used with --flash')
    args = parser.parse_args()
    stage = None

    def remember_stage(_, path):
        nonlocal stage
        stage = path

    try:
        project = resolve_project(args.project)
        openocd = args.openocd or (detect_openocd() if args.flash else None)
        options = Deployment(
            project=project, model=(args.model or project / 'simulink/odrive_foc.slx').resolve(),
            matlab=(args.matlab or detect_matlab() or Path('matlab.exe')).resolve(),
            gcc=(args.gcc or detect_gcc() or compiler()).resolve(),
            output=(args.output or project / 'build/Simulink').resolve(),
            verify_arm=args.verify_arm, create_model=args.create_model, flash=args.flash,
            openocd=openocd.resolve() if openocd else None,
            install_project=args.install_project,
        )
        deploy(options, report=cli_report, on_phase=remember_stage)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError, ImportError) as error:
        cli_report(str(error) + (f'\nLogs: {stage}' if stage else ''))
        parser.exit(1)


if __name__ == '__main__':
    main()
