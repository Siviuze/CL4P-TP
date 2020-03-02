# Run a simulation of claptrap
import numpy as np

from claptrap_simu.core.claptrap import Claptrap
from claptrap_simu.log_handling.viewer3D import display_3d

# Simulation parameters and initial state.
simulation_duration = 5.0
dt = 0.010
x0 = np.zeros((12, 1))
# Start with pitch angle
x0[4, 0] = 0.1

# Claptrap controller: PD on base angle
Kp = 50
Kd = 0.002
def pd_controller(t, q, v):
    # ~ pitch = se3.rpy.matrixToRpy(se3.Quaternion(q[6, 0], q[3, 0], q[4, 0], q[5, 0]).matrix())[1, 0]
    # ~ dpitch = v[4, 0]
    pitch = q[4, 0]
    # ~ dpitch = v[4, 0]
    dpitch = v[4, 0]
    
    u = - np.matrix([[Kp * (pitch + Kd * dpitch)]])
    # -u: torque is applied to the wheel, so the opposite torque is applied to the base.
    return -u 


# Create Claptrap object
claptrap = Claptrap()

# Run simulation, saving it to log file.
output_name="/tmp/sagittal_pd.csv"
claptrap.simulate(x0, simulation_duration, dt, pd_controller, output_name)

# Display simulation
display_3d([output_name])
