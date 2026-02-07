from collections import deque
import numpy as np


class ModifiedSmithPredictor:
    def __init__(self, lambda_v, P1, P2, dt, delay_steps, ctrl=None):
        self.lambda_v = lambda_v
        self.P1 = P1
        self.P2 = P2
        self.dt = dt
        self.ctrl = ctrl  # Optional MNRC controller
        self.model_vel = np.zeros(3)
        self.model_p = np.zeros(3)
        self.delay_buffer = deque(
            [np.zeros(3) for _ in range(delay_steps + 1)], maxlen=delay_steps + 1
        )

    def control(self, r_p, p_m):
        model_delayed = self.delay_buffer[0]
        predicted_p = self.model_p + self.P2 * (p_m - model_delayed)
        e = r_p - predicted_p
        if self.ctrl is not None:
            r_v = self.ctrl.control(predicted_p, r_p)
        else:
            r_v = self.P1 * e
        return r_v

    def update(self, r_v):
        model_vel_dot = -self.lambda_v * (self.model_vel - r_v)
        self.model_vel += self.dt * model_vel_dot
        self.model_p += self.dt * self.model_vel
        self.delay_buffer.append(self.model_p.copy())
