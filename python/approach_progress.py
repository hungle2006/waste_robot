"""Measure net visual change across a bounded window of forward pulses.

This is visual evidence, not odometry. Reacquisition never resets the budget.
"""
from statistics import median


class ForwardProgress:
    def __init__(self, steps, minimum):
        self.steps, self.minimum = steps, minimum
        self.samples = []

    @staticmethod
    def vector(obj):
        return [obj['extent']['rw'], obj['extent']['rh'], obj['bbox_norm'][3]]

    def observe(self, before, after):
        if not self.samples:self.samples.append(self.vector(before))
        self.samples.append(self.vector(after))
        if len(self.samples) <= self.steps:return None
        # Endpoint pairs suppress single-frame box jitter; never sum positive-only noise.
        start=[median(row[i] for row in self.samples[:2]) for i in range(3)]
        end=[median(row[i] for row in self.samples[-2:]) for i in range(3)]
        gains=[b-a for a,b in zip(start,end)]
        result={'confirmed':max(gains)>=self.minimum, 'steps':len(self.samples)-1,
                'start':start, 'end':end, 'gains':gains, 'minimum':self.minimum}
        self.samples=[self.vector(after)]
        return result
