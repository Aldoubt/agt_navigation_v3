"""Load the audited legacy manager source without using an installed copy.

`ament_python` may leave an older copied Python module in ``build/`` when the
legacy package itself has no source change.  Patch 1 parity must instead use
the manager source file currently under review.
"""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sys


def _legacy_manager_path() -> Path:
    names = ('navigation', 'localization', 'agt_localization_manager',
             'agt_localization_manager', 'localization_manager.py')
    search_roots = [Path(__file__).resolve(), Path.cwd().resolve()]
    for root in search_roots:
        for parent in (root, *root.parents):
            candidate = parent.joinpath(*names)
            if candidate.is_file():
                return candidate
            candidate = parent / 'agt_localization_manager' / 'agt_localization_manager' / 'localization_manager.py'
            if candidate.is_file():
                return candidate
    raise RuntimeError('Cannot locate legacy agt_localization_manager source for parity test')


_SPEC = spec_from_file_location('agt_localization_manager_source_oracle', _legacy_manager_path())
if _SPEC is None or _SPEC.loader is None:
    raise RuntimeError('Cannot load legacy localization manager source oracle')
legacy_manager = module_from_spec(_SPEC)
sys.modules[_SPEC.name] = legacy_manager
_SPEC.loader.exec_module(legacy_manager)
