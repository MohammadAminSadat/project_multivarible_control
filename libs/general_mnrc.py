import numpy as np


class GeneralMNRC:
    def __init__(self, f_func, g_func, Lambda_ref, zeta, tau, dt, n=3):
        self.f_func = f_func
        self.g_func = g_func
        self.Lambda = np.diag(Lambda_ref)
        self.k = np.diag(2 * zeta / tau)
        self.k_I = np.diag(1 / (tau**2))
        self.dt = dt
        self.y_m = np.zeros(n)
        self.e_I = np.zeros(n)

    def control(self, y, r, **kwargs):
        dot_y_m = -self.Lambda @ (self.y_m - r)
        f = self.f_func(y)
        g = self.g_func(y, **kwargs)
        inv_g = np.linalg.inv(g)
        e = self.y_m - y
        u = inv_g @ (dot_y_m - f + self.k @ e + self.k_I @ self.e_I)

        self.y_m += self.dt * dot_y_m
        self.e_I += self.dt * e

        return u
