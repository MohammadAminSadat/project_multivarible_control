import numpy as np
from collections import deque


class Simulator:
    def __init__(self, params, omega0=np.zeros(3), p0=np.zeros(3), vel0=np.zeros(3)):
        self.prms = params.copy()
        self.prms["alpha"] = self.prms["delta"] - self.prms["psi"]

        MC = np.zeros((3, 3))
        for i in range(3):
            MC[i, 0] = -np.cos(self.prms["alpha"][i])
            MC[i, 1] = -np.sin(self.prms["alpha"][i])
            MC[i, 2] = self.prms["L"][i] * np.sin(self.prms["psi"][i])
        self.MC = MC

        self.MM = np.diag([1 / self.prms["m"], 1 / self.prms["m"], 1 / self.prms["Jv"]])

        r, kt, kb, ra, ngb = (
            self.prms["r"],
            self.prms["kt"],
            self.prms["kb"],
            self.prms["ra"],
            self.prms["ngb"],
        )

        self.MA = np.eye(3) * (kt * ngb / (ra * r))
        self.MB = np.eye(3) * (kt * kb * (ngb**2) / (ra * r))

        self.M1 = (1 / r) * MC @ self.MM @ MC.T @ self.MA
        self.M2 = (1 / r) * MC @ self.MM @ MC.T @ self.MB

        self.M3 = np.diag([15.0, 15.0, 15.0])  # lambda_w
        self.M4 = r * np.linalg.inv(self.MC)

        self.omega = omega0.copy()
        self.p = p0.copy()
        self.vel = vel0.copy()

        self.delay_steps = int(self.prms["theta"] / self.prms["dt"])
        self.vel_history = deque(
            [np.zeros(3) for _ in range(self.delay_steps + 1)],
            maxlen=self.delay_steps + 1,
        )
        self.p_history = deque(
            [np.zeros(3) for _ in range(self.delay_steps + 1)],
            maxlen=self.delay_steps + 1,
        )

    def rotation(self, phi):
        c = np.cos(phi)
        s = np.sin(phi)
        return np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])

    def inner_step(self, v):
        omega_dot = self.M1 @ v - self.M2 @ self.omega
        self.omega += self.prms["dt"] * omega_dot

    def vehicle_step(self):
        phi = self.p[2]
        R = self.rotation(phi)
        self.vel = R.T @ np.linalg.inv(self.MC) @ (self.prms["r"] * self.omega)
        self.p += self.prms["dt"] * self.vel

        self.vel_history.append(self.vel.copy())

        self.p_history.append(self.p.copy())
