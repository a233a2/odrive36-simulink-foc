"""Install isolated emulator test wheels in tmp, validating PyPI sha256."""
import hashlib
import io
import json
from pathlib import Path
import urllib.request
import zipfile

dest=Path(__file__).resolve().parents[1]/'tmp/testdeps'
for package,version,ending in [('unicorn','2.1.4','win_amd64.whl'),('pyelftools','0.32','none-any.whl')]:
    with urllib.request.urlopen('https://pypi.org/pypi/{}/{}/json'.format(package,version),timeout=30) as response:
        data=json.load(response)
    item=next(x for x in data['urls'] if x['filename'].endswith(ending))
    with urllib.request.urlopen(item['url'],timeout=45) as response:
        wheel=response.read()
    assert hashlib.sha256(wheel).hexdigest()==item['digests']['sha256']
    with zipfile.ZipFile(io.BytesIO(wheel)) as archive:
        for item_info in archive.infolist():
            if '..' in Path(item_info.filename).parts or Path(item_info.filename).is_absolute():
                raise ValueError('Unsafe archive path')
        archive.extractall(dest)
    print(package+'=='+version)
