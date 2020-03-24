# Validate simulation dynamics on a pure sagittal motion
# by comparing it with the analytical equations of the 2D motion.

import numpy as np
import meshcat
import matplotlib.pyplot as plt

from claptrap_simu.core.claptrap import Claptrap
from claptrap_simu.core.sagittal_dynamics import ClaptrapSagittalDynamics

# PD controller parameters
Kp = 200
Kd = 0.1

def claptrap_pd(t, q, v):
    pitch = q[4]
    dpitch = v[4]

    u = Kp * (pitch + Kd * dpitch )
    return u

def analytical_pd(t, x):
    theta = x[0]
    dtheta = x[2]
    return  Kp * (theta + Kd * dtheta )

if __name__ == "__main__":
    # Parameters
    simulation_duration = 1.0
    dt = 0.010

    # Initial state
    x0_claptrap = np.zeros(12)
    x0_claptrap[2] = np.random.rand(1) # Random yaw angle, this should have no impact on simulation
    x0_claptrap[4] = np.random.rand(1) # Random initial pitch angle

    # Initialize analytical simulation
    x0 = np.array([x0_claptrap[4], 0.0, 0.0, 0.0, 0.0])

    # Run pinocchio-based simulation
    claptrap = Claptrap()
    t_claptrap, x_claptrap_full = claptrap.simulate(x0_claptrap, simulation_duration, dt, claptrap_pd)

    # Reformat x_claptrap to match 2D simulation order.
    x_claptrap = [x_claptrap_full[:, 4],    # theta
                  x_claptrap_full[:, 5],    # phi
                  x_claptrap_full[:, 10],   # dtheta
                  x_claptrap_full[:, 11],   #dphi
                  np.cos(x_claptrap_full[:, 2]) * x_claptrap_full[:, 0] + np.sin(x_claptrap_full[:, 2]) * x_claptrap_full[:, 1]] # Longitudinal displacement (x), taking into account the yaw rotation.

    # Run 2D simulation.
    analytical_simulator = ClaptrapSagittalDynamics()
    t_analytical, x_analytical, _ = analytical_simulator.run_nonlinear_simulation(x0, dt, simulation_duration, motor_control_law = analytical_pd)

    # Compare results
    assert (t_claptrap == t_analytical)
    assert np.allclose(np.array(x_claptrap).T, np.array(x_analytical), 1e-5)
    print("Unit test passed, claptrap simulation matches analytical equations for the sagittal motion")
