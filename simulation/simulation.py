# Run a simulation of claptrap
from logger import Logger
from claptrap import Claptrap, CLAPTRAP_STATE_SUFFIXES
import sys
import pinocchio as se3
import numpy as np
import meshcat
from meshcat.animation import Animation

# Parameters
# ~ simulation_length = 20.0
# ~ simulation_length = 5.0
# ~ simulation_length = 5.0
simulation_length = 10.0
dt = 0.010
refresh_rate = 2 # Refresh the display every n itertion.

integral = 0
old_t = 0

# Dummy controller: simpy return zero torque
def zero_torque_controller(t, q, v, q_ref, v_ref, a_ref):
    return np.zeros((1,1)) 
    
# PD controller on base angle
Kp = 150
Kd = 0.01
def pd_controller(t, q, v, q_ref, v_ref, a_ref):
    pitch = se3.rpy.matrixToRpy(se3.Quaternion(q[6, 0], q[3, 0], q[4, 0], q[5, 0]).matrix())[1, 0]
    dpitch = v[4, 0]
    
    u = - np.matrix([[Kp * (pitch + Kd * dpitch)]])
    # -u: torque is applied to the wheel, so the opposite torque is applied to the base.
    return -u 


# Create the viewer.
viewer = meshcat.visualizer.Visualizer("tcp://127.0.0.1:6000")
# Display the simulation as a pure animation after the simulation, so the display can be to be real time.
anim = Animation(default_framerate = 1 / (refresh_rate * dt))

# Initialize claptrap object
claptraps = {}
claptraps["ZeroTorque"] = Claptrap(zero_torque_controller, meshcat_viewer=viewer, meshcat_name = "ZeroTorque", robot_color = np.array([1.0, 0.0, 0.0, 0.5]))
claptraps["PD"] = Claptrap(pd_controller, meshcat_viewer=viewer, meshcat_name = "PD", robot_color = np.array([0.0, 1.0, 0.0, 1.0]))

# Set initial state
quat = se3.Quaternion(se3.rpy.rpyToMatrix(np.matrix([[0.0, 0.2, 0.0]]).T))
for c in claptraps:
    claptraps[c].q[3:7, 0] = quat.coeffs()
    claptraps[c].q[:3, 0] = claptraps[c].wheel_radius
    claptraps[c].solver.set_initial_value(np.concatenate((claptraps[c].q, claptraps[c].v)))

# Initialize logger
logged_values = []
for c in claptraps:
    logged_values += [c + "." +  s for s in CLAPTRAP_STATE_SUFFIXES]
   
logger = Logger(logged_values)

n_iter = 0
t = dt
success = True

while success and t < simulation_length:
    success = True
    n_iter = n_iter + 1
    for c in claptraps:
        success &= claptraps[c].integrate(t)
        
        # Log
        claptraps[c].log_state(logger, "ZeroTorque.")
            
        # Update display
        if n_iter % refresh_rate == 0:
            with anim.at_frame(viewer, n_iter / refresh_rate) as frame:
                claptraps[c].robot.viz.viewer = frame
                claptraps[c].robot.display(claptraps[c].q)
    
    logger.set("time", t)
    t += dt
    logger.new_line()
    sys.stdout.write('Running simulation: {:0.1f}/{}s\r'.format(t, simulation_length))
    sys.stdout.flush()

logger.save('/tmp/claptrap_simulation.csv')

# Display animation
viewer.set_animation(anim)
