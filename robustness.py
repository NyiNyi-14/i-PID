# %%
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from collections import deque

import os
print(os.getcwd())
os.chdir("/Users/nyinyia/Documents/09_LSU_GIT/i_PID copy/Libraries_NNA")
print(os.listdir())

# Local
from iPID_controller import iPID_controller
from DC_Motor import DC_motor
from InputShaping import InputShaping

# %% Closed loop with noise and delay
class Closed_loop:
    def __init__(self, system, controller, noise_sigma=0.0, delay_steps=0):
        self.system = system
        self.controller = controller
        self.noise_sigma = noise_sigma
        self.delay_steps = delay_steps
        self.buffer = deque([0.0]*delay_steps, maxlen=delay_steps)

    def simulate_extra(self, time, dt, ref, init, control_index=2, extra_args_func=None):
        state = np.array(init, dtype=float)
        states = []

        if np.isscalar(ref):
            ref = np.full_like(time, ref)

        for i, t in enumerate(time):
            mea = state[control_index]
            mea += np.random.normal(0, self.noise_sigma)
            self.buffer.append(mea)
            if self.delay_steps > 0:
                mea_used = self.buffer[0]
            else:
                mea_used = mea

            u = self.controller.control(mea_used, ref[i], dt)

            extra_args = extra_args_func[i] if extra_args_func is not None else ()
            sol = solve_ivp(self.system.ODE, [t, t + dt], state, args=(u, extra_args))
            state = sol.y[:, -1]
            states.append(state)

        self.controller.reset()
        return time, np.array(states)

# %% Machine parameters
motor_params = {
    "R": (0.2, 0.1),
    "L": (0.5, 0.002),
    "Kb": (1.0, 0.005),
    "Kt": (1.0, 0.005),
    "J": (2.0, 0.002),
    "B": (0.05, 0.0002)
}

omega = 0
ia = 0
init = [ia, omega]

dt = 0.01  
duration = 100 
time = np.arange(0, duration, dt)
omega_ref = 80.0  
TL = 100 * np.ones_like(time)  
TL[:int(duration/dt/1.4)] = 0

motor_test = DC_motor(motor_params["R"][0], 
                 motor_params["L"][0], 
                 motor_params["Kb"][0], 
                 motor_params["Kt"][0], 
                 motor_params["J"][0],
                 motor_params["B"][0],
                 TL = TL)

# %% Controller parameters and Simulation
zeta = 0.0 # look at MATLAB Root Locus
omega_n = 2.31 # look at MATLAB Root Locus
IS = InputShaping(zeta, omega_n)

Kp = 3.33648935332675
Ki = 0.656107950597057
Kd = -0.0141651780403183
iPID = iPID_controller(Kp, Ki, Kd)

fix_t_switch = 1.175
i_robust_is = 80 * IS.robust_is(time, fix_t_switch)

fix_L_switch = 1.170
i_pre = IS.robust_is(time, fix_L_switch)
i_robust_Lis = np.zeros_like(i_pre)
i_robust_Lis[int(duration/dt/1.4):] = TL[-1] * i_pre[:len(time) - int(duration/dt/1.4)]

noise_sigma = [0.5, 1, 2, 10]
delay_steps = [int(0.1 / dt), int(0.1 / dt), int(0.1 / dt), int(0.1 / dt)]
i_state = []
i_state_robust = []

for i,j in zip(noise_sigma, delay_steps):
    print(f"Noise sigma: {i}, Delay steps: {j}")
    iPID_loop = Closed_loop(motor_test, iPID, noise_sigma=i, delay_steps=j)
    time, state = iPID_loop.simulate_extra(time, dt, omega_ref, init = init, control_index = 1, extra_args_func = TL)
    time, robust = iPID_loop.simulate_extra(time, dt, i_robust_is, init = init, control_index = 1, extra_args_func = i_robust_Lis)
    i_state.append(state)
    i_state_robust.append(robust)

plt.figure(figsize=(10, 4))
for idx, (i, j, state) in enumerate(zip(noise_sigma, delay_steps, i_state)):
    plt.subplot(2, 2, idx+1)
    plt.plot(time, omega_ref * np.ones_like(time), label="Reference Speed (rad/s)")
    plt.plot(time, state[:, 1], label=f"iPID (rad/s)\nNoise: {i}, Delay steps: {j}")
    plt.plot(time, i_state_robust[idx][:, 1], label=f"iPID Robust IS (rad/s)\nNoise: {i}, Delay steps: {j}")
    plt.xlabel("Time (s)")
    plt.ylabel("Motor Speed (rad/s)")
    plt.grid()

# %%
