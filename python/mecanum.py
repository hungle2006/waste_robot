"""X-roller Mecanum. Axes: forward, robot-right, clockwise; PWM is raw driver sign."""

DRIVE_AXES = {
    "FORWARD": (100, 0, 0), "BACK": (-100, 0, 0),
    "LEFT": (0, -100, 0), "RIGHT": (0, 100, 0),
    "FORWARD_LEFT": (100, -100, 0), "FORWARD_RIGHT": (100, 100, 0),
    "BACK_LEFT": (-100, -100, 0), "BACK_RIGHT": (-100, 100, 0),
    "TURN_LEFT": (0, 0, -100), "TURN_RIGHT": (0, 0, 100),
    "ARC_FORWARD_LEFT": (100, 0, -50), "ARC_FORWARD_RIGHT": (100, 0, 50),
    "ARC_BACK_LEFT": (-100, 0, 50), "ARC_BACK_RIGHT": (-100, 0, -50),
}
DEFAULT_SPEED = 20
DRIVE_LEASE_MS = 500
DRIVE_PROTOCOL = 4
SMOOTH_PROTOCOL = 1


def smooth_timing(equivalent_ms, ramp_cap_ms):
    """Keep nominal PWM-time area: two smoothstep ramps equal one full ramp interval."""
    if type(equivalent_ms) is not int or not 50<=equivalent_ms<=400:
        raise ValueError('Sai thời lượng nhịp mềm')
    if type(ramp_cap_ms) is not int or not 40<=ramp_cap_ms<=400:
        raise ValueError('Sai thời gian ramp')
    ramp=min(equivalent_ms,ramp_cap_ms)
    hold=equivalent_ms-ramp
    return ramp,hold,2*ramp+hold


def smooth_pwm(peak,elapsed_ms,ramp,hold):
    if not (1<=peak<=255 and 40<=ramp<=400 and 0<=hold<=400 and 2*ramp+hold<=1000):return 0
    if elapsed_ms>=2*ramp+hold:return 0
    if ramp<=elapsed_ms<ramp+hold:return peak
    t=elapsed_ms if elapsed_ms<ramp else 2*ramp+hold-elapsed_ms
    q=max(0,int(t)*1000//ramp)
    eased=q*q*(3000-2*q)//1000000
    return (peak*eased+500)//1000


def mix_mecanum(forward, right, clockwise, speed=DEFAULT_SPEED):
    for axis in (forward, right, clockwise):
        if type(axis) is not int or not -100 <= axis <= 100:
            raise ValueError("Trục Mecanum phải là số nguyên trong -100..100")
    if type(speed) is not int or not 1 <= speed <= 255:
        raise ValueError("PWM bánh xe phải là số nguyên trong 1..255")
    # FR=M1(-), RL=M2(-), RR=M3(-), FL=M4(+).
    raw = [-(forward-right-clockwise), -(forward-right+clockwise),
           -(forward+right-clockwise), forward+right+clockwise]
    scale = max(100, *(abs(value) for value in raw))
    return [(1 if value >= 0 else -1) * ((abs(value)*speed+scale//2)//scale)
            for value in raw]
