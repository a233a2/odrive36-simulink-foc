"""Fetch the small, pinned CMSIS subset; no HAL or ODrive firmware is used."""
import hashlib
import json
from pathlib import Path
import urllib.request
import io
import zipfile

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / 'vendor' / 'manifest.json'

def get(url):
    req = urllib.request.Request(url, headers={'User-Agent': 'CM-Odrive-build'})
    with urllib.request.urlopen(req, timeout=45) as response:
        return response.read()

def main():
    old = json.loads(MANIFEST.read_text()) if MANIFEST.exists() else {}
    pins = old.get('pins', {})
    repo = 'STMicroelectronics/cmsis-device-f4'
    if repo not in pins:
        pins[repo] = json.loads(get('https://api.github.com/repos/' + repo + '/commits/master'))['sha']
    pins['ARM-software/CMSIS_5'] = '5.9.0'
    files = []
    for name in ['stm32f4xx.h', 'stm32f405xx.h', 'system_stm32f4xx.h']:
        files.append((repo, 'Include/' + name, 'vendor/Device/' + name))
    files += [(repo, 'Source/Templates/gcc/startup_stm32f405xx.s', 'startup/startup_stm32f405xx.s'),
              (repo, 'LICENSE.md', 'vendor/Device/LICENSE.md')]
    for name in ['core_cm4.h', 'cmsis_gcc.h', 'cmsis_compiler.h', 'cmsis_version.h', 'mpu_armv7.h']:
        files.append(('ARM-software/CMSIS_5', 'CMSIS/Core/Include/' + name, 'vendor/CMSIS/' + name))
    files.append(('ARM-software/CMSIS_5', 'LICENSE.txt', 'vendor/CMSIS/LICENSE.txt'))
    entries = []
    for repo_name, source, target in files:
        url = 'https://raw.githubusercontent.com/{}/{}/{}'.format(repo_name, pins[repo_name], source)
        path = ROOT / target
        path.parent.mkdir(parents=True, exist_ok=True)
        if not path.exists():
            path.write_bytes(get(url))
        entries.append({'path': target, 'url': url, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
        print(target)
    # Keep the CDC/DWC2 subset, not TinyUSB examples, host stack or other MCUs.
    dest = ROOT / 'vendor/tinyusb'
    if not (dest / 'src/tusb.c').exists():
        archive = zipfile.ZipFile(io.BytesIO(get('https://codeload.github.com/hathach/tinyusb/zip/refs/tags/0.17.0')))
        prefixes = ['src/common/', 'src/device/', 'src/osal/', 'src/class/cdc/', 'src/portable/synopsys/dwc2/']
        for info in archive.infolist():
            rel = info.filename.split('/', 1)[-1]
            if info.is_dir() or '..' in Path(rel).parts:
                continue
            if rel in ['LICENSE', 'src/tusb.c', 'src/tusb.h', 'src/tusb_option.h'] or any(rel.startswith(p) for p in prefixes):
                out = dest / rel
                out.parent.mkdir(parents=True, exist_ok=True)
                out.write_bytes(archive.read(info))
        print('TinyUSB 0.17.0 CDC/DWC2 subset')
    pins['hathach/tinyusb']='0.17.0'
    for path in sorted(dest.rglob('*')):
        if path.is_file():
            rel=path.relative_to(dest).as_posix()
            entries.append({'path':path.relative_to(ROOT).as_posix(),
                            'url':'https://raw.githubusercontent.com/hathach/tinyusb/0.17.0/'+rel,
                            'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    MANIFEST.write_text(json.dumps({'pins': pins, 'files': entries}, indent=2) + '\n')

if __name__ == '__main__':
    main()
