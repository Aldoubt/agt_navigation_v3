"""Flatten this task's saved image without materializing its filesystem on host."""
import copy
import io
import json
from pathlib import Path
import sys
import tarfile

archive = tarfile.open(sys.argv[1], 'r:')
manifest = json.load(archive.extractfile('manifest.json'))[0]
entries = {}
layers = []
for name in manifest['Layers']:
    layer = tarfile.open(fileobj=archive.extractfile(name), mode='r:')
    layers.append(layer)
    members = layer.getmembers()
    for item in members:
        path = item.name.removeprefix('./').rstrip('/')
        parent, _, base = path.rpartition('/')
        if base.startswith('.wh.'):
            target = parent if base == '.wh..wh..opq' else (parent + '/' if parent else '') + base[4:]
            for old in list(entries):
                if old.startswith(target + '/') or (base != '.wh..wh..opq' and old == target):
                    del entries[old]
    for item in members:
        path = item.name.removeprefix('./').rstrip('/')
        if not path or path.rsplit('/', 1)[-1].startswith('.wh.'):
            continue
        assert not path.startswith('/') and '..' not in path.split('/')
        entries[path] = (layer, item)
code_roots = ('opt/nav_ws', 'opt/mapping_ws', 'opt/hmi', 'opt/hmi_build', 'opt/hmi_field_tests', 'opt/agt')
overrides = {
    'opt/nav_ws/src/agt_navigation_v3/appliance/Dockerfile': Path('/workspace/agt_navigation_v3/appliance/Dockerfile').read_bytes(),
    'opt/nav_ws/src/agt_navigation_v3/appliance/scripts/entrypoint.sh': Path('/workspace/agt_navigation_v3/appliance/scripts/entrypoint.sh').read_bytes(),
}
def order(path):
    item = entries[path][1]
    return (0 if item.isdir() else 3 if item.islnk() else 2 if item.issym() else 1, path.count('/') if item.isdir() else 0, path)
output = tarfile.open(fileobj=sys.stdout.buffer, mode='w|')
for path in sorted(entries, key=order):
    layer, original = entries[path]
    item = copy.copy(original)
    item.name = path
    if any(path == root or path.startswith(root + '/') for root in code_roots):
        item.mode |= 0o555 if item.isdir() or item.mode & 0o111 else 0o444
    if path in overrides:
        data = overrides[path]
        item.size = len(data)
        output.addfile(item, io.BytesIO(data))
    else:
        output.addfile(item, layer.extractfile(original) if item.isreg() else None)
output.close()
print('Flattened entries: ' + str(len(entries)), file=sys.stderr)
