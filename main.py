import numpy as np
from math import cos as cos, sin as sin
from kinematic_solver import Kinematics
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def animated_plot():
    fig = plt.figure()
    ax = plt.axes(xlim=(-2., 10.), ylim=(-2., 2.))
    time_text = ax.text(8, -1.5, '')
    x_text = ax.text(8, -1.65, '')
    y_text = ax.text(8, -1.8, '')
    times = np.linspace(0., t_final, int(t_final/dt)+1)
    xdata = []
    ydata = []

    def update(i):
        time_text.set_text("Time: {0:0.2f}".format(times[i]))
        x_text.set_text("X: {0:0.2f}".format(x_positions[i]))
        y_text.set_text("Y: {0:0.2f}".format(y_positions[i]))
        xdata.append(x_positions[i])
        ydata.append(y_positions[i])
        line.set_data(xdata, ydata)
        curr_line.set_data(x_positions[i], y_positions[i])
        return (line,) + (curr_line,) + (time_text,) + (x_text,) + (y_text,)

    line, = plt.plot([], [], 'b-')
    curr_line, = plt.plot([], [], 'bx')
    plt.xlabel('X position (m)')
    plt.ylabel('Y position (m)')
    line_ani = animation.FuncAnimation(fig, update, int(t_final/dt)+1, interval=1, repeat=True)
    plt.show()


if __name__ == "__main__":
    x = 0.  # Starting x position
    xr = 0.  # Desired x position

    y = 1.  # Starting y position
    yr = 0.  # Desired y position

    phi = 0.  # Starting heading
    phi_r = 0.  # Desired heading

    init_xe = cos(phi)*(xr-x) + sin(phi)*(yr-y)
    init_ye = -sin(phi)*(xr-x) + cos(phi)*(yr-y)
    init_phi_e = phi_r - phi

    # Lyapunov controller gains
    kx = 10.
    ky = 10.
    k_phi = 10.
    kinematics_solver = Kinematics(kx, ky, k_phi, xe=init_xe, ye=init_ye, phi_e=init_phi_e)

    t = 0.
    t_final = 5.
    dt = .001
    x_positions = [x]
    y_positions = [y]
    v_des = 2.
    w_des = 0.
    while t <= t_final:
        # Random noise parameters to be added to controller commands
        x_noise = np.random.uniform(-2., 2.)
        y_noise = np.random.uniform(-2., 2.)
        phi_noise = np.random.uniform(-.5, .5)

        kinematics_solver.solve_step(dt, v_des, w_des, x_noise, y_noise, phi_noise)
        phi += kinematics_solver.w*dt
        x += kinematics_solver.v*cos(phi)*dt
        y += kinematics_solver.v*sin(phi)*dt
        x_positions.append(x)
        y_positions.append(y)
        t += dt

    print('FINAL POSITION', ' X: ', round(x, 2), ' Y: ', round(y, 2), ' PHI: ', round(phi, 2))
    kinematics_solver.plot_results()

    animated_plot()
