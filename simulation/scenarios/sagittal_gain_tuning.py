# Tune a linear controler to stabilize the sagittal motion of Claptrap.
# Controller gain tuning is done using a LQR on the linear system ; the simulationr result is then
# compared with the nonlinear dynamics.

import numpy as np
import control
import argparse

from claptrap_simu.core.claptrap import Claptrap
from claptrap_simu.core.sagittal_dynamics import ClaptrapSagittalDynamics
from claptrap_simu.log_handling.viewer3D import display_3d

parser = argparse.ArgumentParser(description="Tune the gains of the controller using LQR on the linearized dynamics.")
parser.add_argument("-fb", "--fullbody", required=False, action="store_true", help="If set, run 3D simulation and display results in Meshcat")
parser.add_argument("-nt", "--no_test", required=False, action="store_true", help="If set, don't perform 'test' simulation.")
args = parser.parse_args()

simulator = ClaptrapSagittalDynamics()

# Simulation parameters
simulation_duration = 15
dt = 0.010

# LQR weights: cost is x^T R x + u^T Q u
# We consider the following cost: theat **2 + alhpa * dtheta **2 + beta * phi**2 + gamma * dphi **2
# where phi = theta + psi = x[0] + x[2] is the total travel angle.
alpha = 0.1
beta = 0.01
gamma = 0.05

R = np.array([[1 + beta, beta, 0, 0],
              [beta, beta, 0, 0],
              [0, 0, alpha + gamma, gamma],
              [0, 0, gamma, gamma]])
q = 1.0
# Get A, B matrices.
A, B = simulator.get_linear_system_matrices()

# Tune gains using LQR
K_lqr = np.array(control.lqr(A, B, R, np.array([q]))[0])
print(f"LQR gains: {K_lqr}")

# Design a simple PD on theta, for comparison
kp, kd = 25, 0.2
K_pd = -np.array([[kp, 0, kp * kd, 0]])
K_pd[0, 1] = 0
K_pd[0, 3] = 0
print(f"Comparison PD gains: {K_pd}")


if not args.no_test:
    # Run a 2D simulation using linear and non-linear model, for LQR and benchmarking PD, and display the curves.
    x0 = np.array([0.1, 0.0, 0.0, 0.0])
    sol_lin_lqr = simulator.run_linear_simulation(x0, dt, simulation_duration, K_lqr)
    sol_nl_lqr = simulator.run_nonlinear_simulation(x0, dt, simulation_duration, K_lqr)
    sol_pd = simulator.run_linear_simulation(x0, dt, simulation_duration, K_pd)
    
    # Plot results
    simulator.plot([sol_lin_lqr, sol_nl_lqr, sol_pd], ["LQR, linear", "LQR, non-linear", "PD, linear"])

if args.fullbody:
    # Run a 3D simulation using the full model.
    simulation_duration = 10.0
    dt = 0.010
    x0 = np.zeros(12)
    x0[4] = 0.1
    
    class LinearController:
        def __init__(self, K):
            self.K = K
        
        def compute(self, t, q, v):
            # Extract state: theta, phi, dtheta, dphi
            x = np.array([q[4], q[5], v[4], v[5]])
            return - self.K @ x

    claptrap = Claptrap()
    controller = LinearController(K_pd)
    claptrap.simulate(x0, simulation_duration, dt, controller.compute, "/tmp/sagittal_pd.csv")
    controller = LinearController(K_lqr)
    claptrap.simulate(x0, simulation_duration, dt, controller.compute, "/tmp/sagittal_lqr.csv")
    
    # Display the results in meshcat
    display_3d(["/tmp/sagittal_pd.csv", "/tmp/sagittal_lqr.csv"])

