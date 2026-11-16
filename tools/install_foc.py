"""Publish the generated controller after the STM32 build has passed."""
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile

from build import EIDE_FOC_TARGET, PROJECT_EXPORT, controller_files


def has_eide_target(project):
    if project is None:
        return False
    try:
        config = (project / '.eide/eide.yml').read_text(encoding='utf-8-sig')
    except (OSError, UnicodeError):
        return False
    return (re.search(rf'''(?m)^[ \t]+["']?{EIDE_FOC_TARGET}["']?:[ \t]*$''', config) is not None
            and PROJECT_EXPORT.parent.as_posix() in config and 'CM_USE_SIMULINK_FOC' in config)


@contextmanager
def publication_lock(folder):
    # A process exit releases the OS lock. Keep the file so competing writers
    # always lock the same object, including after an interrupted publication.
    with (folder / '.foc-publish.lock').open('a+b') as handle:
        handle.seek(0)
        try:
            if os.name == 'nt':
                import msvcrt
                msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as error:
            raise RuntimeError('这个工程正在同步另一份生成代码。') from error
        yield


def publish_controller(project, generated, build_dir, manifest):
    destination = project / PROJECT_EXPORT
    folder = project / 'simulink'
    backup = folder / '.foc-previous'
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.parent.resolve() != destination.parent or folder.resolve() != folder:
        raise ValueError('生成代码目录必须位于选定工程内。')

    files = controller_files(generated)
    source_hashes = manifest['generated_sources']
    if {path.name for path in files} != source_hashes.keys():
        raise RuntimeError('生成源码在编译期间发生变化，请重新构建。')
    metadata = {
        'directory': PROJECT_EXPORT.as_posix(),
        'eide_target': EIDE_FOC_TARGET,
        'build_directory': str(build_dir),
        'model_sha256': manifest['artifacts']['odrive_foc.slx'],
        'firmware_sha256': manifest['artifacts']['firmware/CM_Odrive_Simulink.elf'],
        'arm_regression': manifest['arm_regression'],
        'sources': source_hashes,
    }
    with publication_lock(folder), tempfile.TemporaryDirectory(prefix='.foc-stage-', dir=folder) as scratch:
        area = Path(scratch)
        pending = area / 'current'
        pending.mkdir()
        # Build the complete replacement outside the directory EIDE scans.
        for source in files:
            content = source.read_bytes()
            if hashlib.sha256(content).hexdigest() != source_hashes[source.name]:
                raise RuntimeError(f'{source.name} 在编译期间发生变化，请重新构建。')
            (pending / source.name).write_bytes(content)
        (pending / 'deployment.json').write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')

        for active in (destination, backup):
            if active.is_symlink() or (active.exists() and (
                    active.resolve() != active or not (active / 'deployment.json').is_file())):
                raise ValueError(f'此目录已有手动维护的文件，请先移出后再同步。{active}')
        if backup.exists() and not destination.exists():
            backup.rename(destination)
        if backup.exists():
            backup.rename(area / 'superseded')
        if destination.exists():
            destination.rename(backup)
        try:
            pending.rename(destination)
        except BaseException:
            if backup.exists() and not destination.exists():
                backup.rename(destination)
            raise
    return metadata
