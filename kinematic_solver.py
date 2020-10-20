import numpy as np
import matplotlib.pyplot as plt
from math import sin as sin, cos as cos


class Kinematics:
    def __init__(self, kx, ky, k_phi, **init_kwargs):
        self.kx = kx
        self.ky = ky
        self.k_phi = k_phi
        init_xe = init_kwargs['xe']
        init_ye = init_kwargs['ye']
        init_phi_e = init_kwargs['phi_e']
        self.u = np.zeros((1, 3))
        self.u[0] = np.array([init_xe, init_ye, init_phi_e])
        self.t = 0.
        self.hold_t = [self.t]
        self.count = 0
        self.v = 0.
        self.w = 0.

    @property
    def time(self):
        return round(self.t, 1)

    def controller(self, states, t, vr, wr, x_noise, y_noise, phi_noise):
        """
        Equations from 'Introduction to Mobile Robot Control - Spyros G. Tzafestas (Chap 5., pgs. 160-161)'

        Lyapunov function:
        V(x) = .5(xe^2 + ye^2) + (1 - cos(phi_e))
        """

        xe, ye, phi_e = states

        v = vr*cos(phi_e) + self.kx*xe
        w = wr + vr*self.ky*ye + self.k_phi*sin(phi_e)
        self.v = v
        self.w = w

        xe_dot = vr*cos(phi_e+phi_noise) - v + (ye+y_noise)*w
        ye_dot = vr*sin(phi_e+phi_noise) - (xe+x_noise)*w
        phi_e_dot = wr - w

        return np.array([xe_dot, ye_dot, phi_e_dot])

    def rk4(self, h, vr, wr, x_noise, y_noise, phi_noise):
        self.t += h
        self.hold_t.append(self.t)

        k1 = h * self.controller(self.u[self.count], self.t, vr, wr, x_noise, y_noise, phi_noise)
        k2 = h * self.controller(self.u[self.count] + 0.5 * k1, self.t + 0.5*h, vr, wr, x_noise, y_noise, phi_noise)
        k3 = h * self.controller(self.u[self.count] + 0.5 * k2, self.t + 0.5*h, vr, wr, x_noise, y_noise, phi_noise)
        k4 = h * self.controller(self.u[self.count] + k3, self.t + 0.5*h, vr, wr, x_noise, y_noise, phi_noise)
        next_states = ([self.u[self.count] + (k1 + 2*(k2 + k3) + k4) / 6])
        self.u = np.vstack([self.u, next_states])

        self.count += 1

        return self.u, self.t

    def solve_step(self, dt, vr, wr, x_noise, y_noise, phi_noise):
        u, t = self.rk4(dt, vr, wr, x_noise, y_noise, phi_noise)
        # print('TIME: ', round(t, 3), 'STATES: ', u[-1])
        return u[-1]

    def plot_results(self):
        plt.title('Error States')
        plt.plot(self.hold_t, self.u[:, 0], label='xe')
        plt.plot(self.hold_t, self.u[:, 1], label='ye')
        plt.plot(self.hold_t, self.u[:, 2], label='phi_e')
        plt.legend()
        plt.grid()
        plt.show()
