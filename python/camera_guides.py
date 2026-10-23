"""Display-only AUTO landmarks. Never annotate the image supplied to Modal.

OpenCV uses ASCII labels; the calibration page provides the Vietnamese legend.
Only configuration, controller state and joint feedback are read here. No motion.
"""
import copy
import json
import math
import time
from pathlib import Path
from config_io import read_config
from pose_profile import reference_at
from arm_path import targets_at


def _number(value, low=0., high=1.):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError('Invalid guide coordinate')
    return float(value)


def guide_layout(config):
    """Normalized landmarks, using exactly the configured AUTO thresholds."""
    cal = config['calibration']
    if cal.get('mode', 'calibrated') != 'relative':
        return {'supported': False}
    target = [_number(v) for v in cal['relative_target_norm']]
    tolerance = [_number(v, 0.001, .25) for v in cal['relative_tolerance_norm']]
    if len(target) != 2 or len(tolerance) != 2:
        raise ValueError('Invalid guide target')
    a, v, n = (config.get(k, {}) for k in ('approach', 'visibility', 'near'))
    approach = a.get('enabled', False)
    visibility = approach and v.get('enabled', False)
    metric = config.get('distance', {}).get('enabled', False)
    return {
        'supported': True, 'target': target, 'tolerance': tolerance,
        'base_enabled': approach,
        'base_x': _number(a.get('target_x_norm', .5)),
        'base_tol': _number(a.get('x_tolerance_norm', .03), .001, .25),
        'handoff': _number(v.get('trigger_bottom_norm', .54) if visibility else a.get('stop_bottom_norm', .78)) if approach and not metric else None,
        'distance': config.get('distance', {}) if metric else None,
        'blur': _number(v.get('blur_bottom_norm', .60)) if visibility else None,
        'min_top': _number(v.get('min_top_norm', .03)) if visibility else None,
        'min_bottom': _number(v.get('min_bottom_norm', .15)) if visibility else None,
        'edge': _number(n.get('edge_margin_norm', .01), 0, .05) if n.get('enabled', True) else None,
        'stop_rw': _number(a.get('stop_rw', .40)) if approach else None,
        'stop_rh': _number(a.get('stop_rh', .70)) if approach else None,
        'near_rw': _number(n.get('stop_rw', .60)) if n.get('enabled', True) else None,
        'final': cal['grasp_final_s2_us'],
        'handoff_min_rw': 0 if (config.get('confirmed_grasp') or {}).get('enabled') else a.get('handoff_min_rw',0),
        'handoff_min_rh': 0 if (config.get('confirmed_grasp') or {}).get('enabled') else a.get('handoff_min_rh',0),
        'confirmed_grasp': config.get('confirmed_grasp'),
        'pose_profile':config.get('pose_profile'),
        'arm_path':config.get('arm_path'),
        'box_tof_flow':config.get('box_tof_flow'),
    }


class CameraGuides:
    def __init__(self, path, controller=None, brain=None):
        self.path = Path(path)
        self.controller, self.brain = controller, brain
        self._checked_at = -math.inf
        self._config = None

    def config(self):
        # START snapshots the config. During AUTO, display that same snapshot,
        # not a file edited behind the running controller's back.
        if self.controller and self.controller.running and hasattr(self.controller, 'cfg'):
            return copy.deepcopy(self.controller.cfg)
        now = time.monotonic()
        if now - self._checked_at >= .5:
            self._checked_at = now
            self._config = None  # Never silently retain old thresholds after a bad edit.
            self._config = read_config(self.path)
        if self._config is None:
            raise ValueError('Guide config unavailable')
        return self._config

    def render(self, bgr):
        import cv2
        frame = bgr.copy()  # The raw cache/appearance descriptor must remain unchanged.
        h, w = frame.shape[:2]
        scale = max(.32, min(.65, w / 1500))
        font = cv2.FONT_HERSHEY_SIMPLEX
        stroke = max(1, round(w / 640))
        cyan, green, amber, gray = (255, 220, 0), (90, 235, 90), (0, 195, 255), (180, 180, 180)

        def px(x): return max(0, min(w - 1, round(x * w)))
        def py(y): return max(0, min(h - 1, round(y * h)))

        def label(text, x, y, color=gray):
            # Fit labels to the actual encoded frame, including small cameras.
            while text and cv2.getTextSize(text, font, scale, 1)[0][0] > w - 8:
                text = text[:-1]
            tw, th = cv2.getTextSize(text, font, scale, 1)[0]
            x = max(4, min(w - tw - 4, x))
            y = max(th + 3, min(h - 4, y))
            cv2.rectangle(frame, (x - 2, y - th - 2), (x + tw + 2, y + 3), (15, 15, 15), -1)
            cv2.putText(frame, text, (x, y), font, scale, color, 1, cv2.LINE_AA)

        def dashed(p1, p2, color, dash=10):
            dx, dy = p2[0] - p1[0], p2[1] - p1[1]
            length = max(abs(dx), abs(dy))
            if not length: return
            for start in range(0, length + 1, dash * 2):
                end = min(start + dash, length)
                a = (round(p1[0] + dx * start / length), round(p1[1] + dy * start / length))
                b = (round(p1[0] + dx * end / length), round(p1[1] + dy * end / length))
                cv2.line(frame, a, b, color, stroke)

        state = dict(getattr(self.controller, 'state', {}) or {})
        active = bool(state.get('auto_running'))
        rear = active and (state.get('flow') == 'rear' or state.get('state', '').startswith('auto_rear'))
        try:
            c = guide_layout(self.config())
            if rear:
                label('REAR / D18 - FRONT GUIDES OFF', 8, h - 12, amber)
                return frame
            if not c['supported']:
                label('CALIBRATED MODE - RELATIVE GUIDES OFF', 8, h - 12, amber)
                return frame
            tof=c.get('box_tof_flow') or {}
            if tof.get('enabled'):
                p1=state.get('s2_us',c['arm_path']['start']['p1'])
                if self.brain:
                    p1=self.brain.snapshot().get('hardware',{}).get('joints',{}).get('p1',p1)
                if type(p1) is not int:p1=c['arm_path']['start']['p1']
                p1=max(c['arm_path']['start']['p1'],min(c['final'],p1))
                phase=state.get('phase','box_follow') if active else 'box_follow'
                sensor_only=phase in ('tof_approach','final_ir') or (active and p1>=tof['handoff_p1_us'])
                if not sensor_only:
                    ref=reference_at(c['pose_profile'],p1)
                    for x in (ref['target_x_norm']-ref['x_tolerance_norm'],ref['target_x_norm']+ref['x_tolerance_norm']):
                        dashed((px(x),0),(px(x),h-1),green)
                    for y in (ref['target_bottom_norm']-ref['bottom_tolerance_norm'],ref['target_bottom_norm']+ref['bottom_tolerance_norm']):
                        dashed((0,py(y)),(w-1,py(y)),cyan)
                    cv2.line(frame,(0,py(ref['target_bottom_norm'])),(w-1,py(ref['target_bottom_norm'])),cyan,stroke)
                    label('BOTTOM ABOVE: FORWARD / IN OR BELOW: LOWER',8,py(.08),green)
                    label('EDGE: FORWARD / MATCH 1 FRAME / MISS 3 FRAMES',8,py(.15),amber)
                    label('P1 STEP50 / P10 FOLLOWS PATH',8,h-12,green)
                else:
                    final=phase=='final_ir'
                    label('FINAL / IR D19 ONLY' if final else 'VISION OFF / VL53L1X ONLY',8,py(.08),green)
                    mm=state.get('tof_mm')
                    label('FINAL POSE / WAIT D19' if final else
                          f'VL53 {mm/10:.1f}cm / {tof["tof_min_mm"]/10:g}-{tof["tof_max_mm"]/10:g}cm TO FINAL' if type(mm) is int and state.get('tof_valid') is True else
                          f'WAIT VL53 {tof["tof_min_mm"]/10:g}-{tof["tof_max_mm"]/10:g}cm',8,py(.15),amber)
                    label('D19 REQUIRED TO CLOSE GRIP' if final else
                          f'HOLD {p1} / {targets_at(c["arm_path"],p1)["p10"]} / SLOW FORWARD',8,h-12,green)
                return frame
            reference=c.get('confirmed_grasp') or {}
            if (active and reference.get('enabled') and state.get('phase')=='grasp_contact'
                    and state.get('state') in ('auto_grasp_pose_confirmed','auto_near_wait','auto_near_contact','auto_hold')):
                for x in (reference['target_x_norm']-reference['x_tolerance_norm'],
                          reference['target_x_norm']+reference['x_tolerance_norm']):
                    dashed((px(x),0),(px(x),h-1),green)
                label('CONFIRMED GRASP POSE / TOP CLIP ALLOWED',8,py(.08),green)
                label('X ALIGN + FRESH IMAGES + D19 REQUIRED',8,py(.14),green)
                label('27cm REFERENCE POSE / NOT CURRENT RANGE',8,h-12,amber)
                return frame
            profile=c.get('pose_profile') or {}
            if profile.get('enabled'):
                p1=state.get('s2_us') if active else None
                if self.brain:
                    p1=self.brain.snapshot().get('hardware',{}).get('joints',{}).get('p1',p1)
                if type(p1) is not int:p1=c['arm_path']['start']['p1']
                p1=max(c['arm_path']['start']['p1'],min(c['final'],p1))
                ref=reference_at(profile,p1)
                for x in (ref['target_x_norm']-ref['x_tolerance_norm'],ref['target_x_norm']+ref['x_tolerance_norm']):
                    dashed((px(x),0),(px(x),h-1),green)
                for y in (ref['target_bottom_norm']-ref['bottom_tolerance_norm'],ref['target_bottom_norm']+ref['bottom_tolerance_norm']):
                    dashed((0,py(y)),(w-1,py(y)),cyan)
                cv2.line(frame,(0,py(ref['target_bottom_norm'])),(w-1,py(ref['target_bottom_norm'])),cyan,stroke)
                label(f'POSE {ref["stage_index"]+1} | P1 {p1}us | BOTTLE BOTTOM BAND',8,py(.07),green)
                label(f'LENS {ref["lens_cm"]:.1f}cm PHOTO REFERENCE / NOT LIVE RANGE',8,py(.13),amber)
                label('TOP CLIP ALLOWED' if p1>=profile['top_clip_from_p1'] else 'KEEP BOTTLE TOP VISIBLE',8,py(.19),gray)
                label('WHEELS OFF / FRESH VIEW + D19 TO GRIP' if p1>=profile['max_wheel_p1_us'] else 'MOVE WHEELS -> STOP -> NEW FRAME -> ARM STEP',8,h-12,green)
                return frame
            if c['blur'] is not None:
                y = py(c['blur'])
                region = frame[y:, :]
                # Light display shading; no pixels are written to the raw BGR.
                shade = region.copy()
                shade[:] = amber
                cv2.addWeighted(region, .90, shade, .10, 0, dst=region)
                cv2.line(frame, (0, y), (w - 1, y), amber, stroke + 1)
                label(f'BLUR EDGE y2={c["blur"]:.1%}', 8, y + 18, amber)
            if c['edge']:
                x, y = px(c['edge']), py(c['edge'])
                for p1, p2 in [((x, y), (w - 1 - x, y)), ((x, y), (x, h - 1 - y)),
                               ((w - 1 - x, y), (w - 1 - x, h - 1 - y)),
                               ((x, h - 1 - y), (w - 1 - x, h - 1 - y))]:
                    dashed(p1, p2, gray)
            if c['base_enabled']:
                for x in [c['base_x'] - c['base_tol'], c['base_x'] + c['base_tol']]:
                    dashed((px(x), 0), (px(x), h - 1), cyan)
                label(f'BASE X {c["base_x"]:.0%} +/-{c["base_tol"]:.0%}', px(c['base_x']) + 15, py(.09), cyan)
                if c['handoff'] is not None:
                    y = py(c['handoff'])
                    cv2.line(frame, (0, y), (w - 1, y), cyan, stroke + 1)
                    extra = f' + rw>={c["handoff_min_rw"]:.0%},rh>={c["handoff_min_rh"]:.0%}' if c['handoff_min_rw'] or c['handoff_min_rh'] else ''
                    label(f'BASE -> ARM y2={c["handoff"]:.1%} / VIEW HANDOFF'+extra, 8, y - 7, cyan)
                else:
                    d=c['distance']
                    label('BASE STOP: ESTIMATED CM' if c['distance'].get('calibration',{}).get('version')==2 else 'BASE STOP: MEASURED CM / NOT y2=54%', 8, py(.22), cyan)
                    if d and d.get('calibration') and d.get('stop_forward_cm') and (not active or state.get('phase')=='base_approach'):
                        import numpy as np
                        from floor_distance import project
                        cal=d['calibration'];inv=np.linalg.inv(np.array(cal['matrix_norm_to_cm'])).tolist()
                        xs=[p['floor_cm'][0] for p in cal.get('points',[]) if p['kind']=='fit'] or [-5.,5.]
                        a=project(inv,[min(xs),d['stop_forward_cm']]);b=project(inv,[max(xs),d['stop_forward_cm']])
                        ok,p1,p2=cv2.clipLine((0,0,w,h),(round(a[0]*w),round(a[1]*h)),(round(b[0]*w),round(b[1]*h)))
                        if ok:
                            cv2.line(frame,p1,p2,cyan,stroke+1)
                            label(f'DIST STOP {d["stop_forward_cm"]:g}cm',8,min(p1[1],p2[1])-7,cyan)
                    else:label('DIST: SEARCH ONLY' if d and d.get('calibration') else 'DIST: CALIBRATION REQUIRED',8,py(.27),amber)
            x, y = c['target']; tx, ty = c['tolerance']
            cv2.rectangle(frame, (px(x - tx), py(y - ty)), (px(x + tx), py(y + ty)), green, stroke + 1)
            cv2.line(frame, (px(x) - 7, py(y)), (px(x) + 7, py(y)), green, stroke)
            cv2.line(frame, (px(x), py(y) - 7), (px(x), py(y) + 7), green, stroke)
            label('ARM TARGET', px(x + tx) + 8, py(y - ty) - 6, green)
            for key, text in [('min_top', 'TOP GUARD y1'), ('min_bottom', 'BOTTOM GUARD y2')]:
                if c[key] is not None:
                    dashed((0, py(c[key])), (w - 1, py(c[key])), gray)
                    label(f'{text}={c[key]:.1%}', 8, py(c[key]) + 15)
            status = state.get('state', 'preview')
            label(('AUTO ACTIVE' if active else 'REFERENCE / START TO APPLY') + ' | ' + status, 8, h - 31)
            extra = f'S2 max {c["final"]}us | GRIP requires aligned + D19'
            if c['distance'] and state.get('state')=='auto_base_observe' and type(state.get('distance_cm')) in (int,float):
                extra=f'FLOOR Y {state["distance_cm"]:.1f}cm' + (' / ESTIMATE' if state.get('distance_quality')=='preliminary' else f' +/-{state["distance_uncertainty_cm"]:.1f}cm')
            label(extra, 8, h - 12, green)
        except Exception:
            # A display/config error must never terminate the raw camera feed.
            label('GUIDES UNAVAILABLE - CHECK AUTO CONFIG', 8, h - 12, amber)
        return frame

    def jpeg(self, raw_jpeg):
        import cv2
        import numpy as np
        raw = cv2.imdecode(np.frombuffer(raw_jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if raw is None:
            raise ValueError('Camera JPEG cannot be decoded')
        ok, encoded = cv2.imencode('.jpg', self.render(raw), [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            raise ValueError('Camera guides cannot be encoded')
        return encoded.tobytes()


def display_frame(raw, on_frame=None, guides=None):
    """Always deliver raw pixels to vision BEFORE creating the display copy."""
    if on_frame is not None:
        on_frame(raw)
    return guides.render(raw) if guides is not None else raw
