"""Package the Windows UI with its Python, Tk and ARM replay dependencies."""
import argparse
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile

from build import ROOT
from deploy_session import terminate_process_tree


def check_startup(executable, work):
    # Launch the candidate without Python or development packages on PATH.
    env = os.environ.copy()
    for key in ('PYTHONPATH', 'PYTHONHOME'):
        env.pop(key, None)
    system = Path(env.get('SystemRoot', r'C:\Windows'))
    env['PATH'] = os.pathsep.join(map(str, (system / 'System32', system)))
    with tempfile.TemporaryDirectory(prefix='smoke_', dir=work) as folder:
        cwd = Path(folder)
        job, result = cwd / 'smoke.json', cwd / 'result.json'
        job.write_text(json.dumps({'check_runtime': True, 'result': str(result)}), encoding='utf-8')
        with subprocess.Popen([str(executable), '--self-test', str(job)], cwd=cwd, env=env) as process:
            try:
                exit_code = process.wait(timeout=45)
            except subprocess.TimeoutExpired:
                terminate_process_tree(process)
                raise SystemExit('候选 EXE 启动自检超时，现有发布文件未被替换。')
        if not result.is_file():
            raise SystemExit(f'候选 EXE 启动失败（退出状态 {exit_code}）。')
        report = json.loads(result.read_text(encoding='utf-8'))
        if exit_code or report.get('status') != 'ready' or not report.get('frozen'):
            raise SystemExit(f'候选 EXE 启动自检失败：{report.get("error")}')
        print(f'EXE startup and ARM runtime passed (Python {report["python"]}, Tk {report["tk"]})', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--replay-python', type=Path, required=True,
                        help='Python with a PyInstaller bootloader built using --no-cfg')
    args = parser.parse_args()
    if os.name != 'nt' or struct.calcsize('P') != 8:
        raise SystemExit('此打包入口需要 Windows x64 Python。')
    from pefile import PE

    tools = ROOT / 'tools'
    assets = tools / 'ui_assets'
    dependencies = ROOT / 'tmp/testdeps'
    unicorn_dll = dependencies / 'unicorn/lib/unicorn.dll'
    if not unicorn_dll.is_file() or not (dependencies / 'elftools').is_dir() or not (dependencies / 'serial').is_dir():
        raise SystemExit('运行 python tools/install_test_deps.py 后再打包。')
    icon = assets / 'odrive_foc.ico'
    dist = ROOT / 'build/Tools'
    work = ROOT / 'build/package_gui'
    candidate = work / 'candidate'
    work.mkdir(parents=True, exist_ok=True)
    common = [
        '-m', 'PyInstaller', '--noconfirm', '--onefile', '--noupx', '--distpath', str(candidate),
        '--workpath', str(work), '--specpath', str(work), '--paths', str(dependencies),
        '--exclude-module', 'pkg_resources',
    ]
    worker = candidate / 'ODriveFOC-ARM.exe'
    replay_python = args.replay_python.resolve(strict=True)
    subprocess.run([
        str(replay_python), *common, '--console', '--name', worker.stem,
        # Unicorn loads the architecture plugin and DLL dynamically. Its legacy
        # Python 2 fallback would otherwise pull in the pkg_resources runtime hook.
        '--hidden-import', 'unicorn.unicorn_py3.arch.arm', '--exclude-module', 'unicorn.unicorn_py2',
        '--add-binary', str(unicorn_dll) + os.pathsep + 'unicorn/lib', str(tools / 'test_firmware.py'),
    ], cwd=ROOT, check=True)
    with PE(str(worker), fast_load=True) as image:
        if image.FILE_HEADER.Machine != 0x8664 or image.OPTIONAL_HEADER.DllCharacteristics & 0x4000:
            raise SystemExit('ARM 仿真进程需要以 --no-cfg 编译的 PyInstaller x64 bootloader。')

    subprocess.run([
        sys.executable, *common, '--windowed', '--name', 'ODriveFOC', '--icon', str(icon),
        '--exclude-module', 'unicorn', '--exclude-module', 'elftools',
        '--hidden-import', 'serial.urlhandler.protocol_loop',
        '--add-data', str(worker) + os.pathsep + 'arm',
        '--add-data', str(icon) + os.pathsep + 'ui_assets',
        '--version-file', str(assets / 'version_info.txt'), str(tools / 'deploy_gui.py'),
    ], cwd=ROOT, check=True)
    built = candidate / 'ODriveFOC.exe'
    check_startup(built, work)
    dist.mkdir(parents=True, exist_ok=True)
    output = dist / built.name
    built.replace(output)
    print(output, flush=True)


if __name__ == '__main__':
    main()
