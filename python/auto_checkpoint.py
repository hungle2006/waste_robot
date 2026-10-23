"""Crash-safe, *intent-only* AUTO checkpoint.

The journal is never a measurement of arm location: Arduino's joint values are
commanded pulses rather than encoder feedback.  In particular, a USB reset
MUST NOT turn this into permission to replay a gripper release or motor pulse.
"""
import json
import os
import time
from pathlib import Path

VERSION = 1


class AutoCheckpoint:
    def __init__(self, params_path):
        override = os.environ.get('ROBOT_AUTO_CHECKPOINT', '').strip()
        self.path = Path(override) if override else Path(params_path).resolve().with_name('.auto_checkpoint.json')

    def load(self):
        try:
            data = json.loads(self.path.read_text(encoding='utf-8'))
            if not isinstance(data, dict) or data.get('version') != VERSION:
                return None
            return data if data.get('active') is True else None
        except (OSError, ValueError, TypeError):
            return None

    def write(self, *, state, phase, flow, holding_object, selected_bin, label,
              cycles, rear_cycles, drop_failures, status='running', last_joints=None):
        data = {
            'version': VERSION, 'active': True, 'status': status,
            'updated_at': time.time(), 'state': str(state), 'phase': str(phase),
            'flow': str(flow), 'holding_object': bool(holding_object),
            'selected_bin': selected_bin if selected_bin in (1, 2, 3, 4) else None,
            'label': (label or '')[:120],
            'cycles': max(0, int(cycles)), 'rear_cycles': max(0, int(rear_cycles)),
            'drop_failures': max(0, int(drop_failures)),
            'last_commanded_joints': dict(last_joints or {}),
        }
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(self.path.suffix + '.tmp')
        try:
            with tmp.open('w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.path)
        finally:
            try:
                tmp.unlink(missing_ok=True)
            except OSError:
                pass
        return data

    def suspend(self):
        data = self.load()
        if data is None:
            return None
        self.write(state=data['state'], phase=data['phase'], flow=data['flow'],
                   holding_object=data['holding_object'], selected_bin=data.get('selected_bin'),
                   label=data.get('label'), cycles=data.get('cycles', 0),
                   rear_cycles=data.get('rear_cycles', 0), drop_failures=data.get('drop_failures', 0),
                   status='waiting_com_reconnect', last_joints=data.get('last_commanded_joints'))
        return self.load()

    def clear(self):
        try:
            self.path.unlink(missing_ok=True)
        except OSError:
            pass
