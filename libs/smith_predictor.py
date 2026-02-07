from collections import deque
import numpy as np


class SmithPredictor:
    def __init__(self, lambda_v, dt, delay_steps):
        self.lambda_v = lambda_v
        self.dt = dt
        self.model_vel = np.zeros(3)
        self.delay_buffer = deque(
            [np.zeros(3) for _ in range(delay_steps + 1)], maxlen=delay_steps + 1
        )

    def get_predicted(self, vel_m):
        model_delayed = self.delay_buffer[0]
        predicted = self.model_vel + (vel_m - model_delayed)
        return predicted

    def update(self, r_v):
        model_vel_dot = -self.lambda_v * (self.model_vel - r_v)
        self.model_vel += self.dt * model_vel_dot
        self.delay_buffer.append(self.model_vel.copy())
