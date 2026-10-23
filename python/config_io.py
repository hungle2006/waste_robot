"""Load trusted, locally edited Python parameters or legacy JSON.

Compile source directly so rapid edits do not reuse a timestamp-based .pyc.
Loading configuration never opens serial, camera, or network connections.
"""
import copy
import json
from pathlib import Path


def read_namespace(path):
    path = Path(path).resolve()
    namespace = {'__file__': str(path), '__name__': 'robot_parameters'}
    exec(compile(path.read_text(encoding='utf-8-sig'), str(path), 'exec'), namespace)
    return namespace


def read_config(path, kind='auto'):
    path = Path(path)
    if path.suffix.lower() != '.py':
        return json.loads(path.read_text(encoding='utf-8-sig'))
    name = {'auto': 'AUTO_CONFIG', 'robot': 'ROBOT_CONFIG'}[kind]
    value = read_namespace(path).get(name)
    if not isinstance(value, dict):
        raise ValueError(f'{path.name} cần khai báo {name} = {{...}}')
    return copy.deepcopy(value)
