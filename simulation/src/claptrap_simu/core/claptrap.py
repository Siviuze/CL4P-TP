# Claptrap class
# This class runs a simulation of Claptrap using pinocchio, given a controller as input.

# A controller is defined by the following signature:
# def controller(t, q, v):
#   return tau


import pinocchio as pnc
import numpy as np
import scipy.integrate
from tqdm import tqdm
    
from ..log_handling.logger import Logger


class Claptrap():
    def __init__(self):
        '''
            Init the robot object.
            @param controller Controller callback.
            @param initial_state Optional, initial state as tuple (q0, v0) to start the simulation (position and velocity)
            @param meshcat_viewer Optional, meshcat viewer. If set, this viewer can be used to display the robots during simulation.
            @param meshcat_name Optional, if meshcat_viewer is set, name of the robot in the viewer.
            @param robot_color Optional, if meshcat_viewer is set, color of the robot in the viewer.
        '''
        
        # TODO: fix import of URDF !
        import os
        import claptrap_simu
        urdf_path = os.path.join(os.path.dirname(claptrap_simu.__file__),'../data/claptrap.urdf')
        
        self.robot = pnc.RobotWrapper.BuildFromURDF("data/claptrap.urdf", ["data/"], root_joint=None)
        
        # Compute wheel radius vector.
        self.wheel_radius = self.robot.model.jointPlacements[self.robot.model.getJointId("BodyJoint")].translation[2, 0]
        
    
    def _dynamics(self, t, x):
        ''' Forward dynamics of the robot, to integrate
        '''
        # Split input as (q, v) pair
        q = np.matrix(x[:self.robot.nq]).T
        v = np.matrix(x[self.robot.nq:]).T
        # Renormalize quaternion to prevent propagation of rounding errors due to integration.
        # ~ q[3:7, 0] /= np.linalg.norm(q[3:7, 0])
        
        # Run forward dynamic computation
        # Compute H and g + coloriolis effects
        H =  self.robot.mass(q)
        g = self.robot.nle(q, v)
        
        # Compute contact jacobian and derivative (drift).
        # Since q = (x y gamma beta alpha theta) where (alpha beta gamma) are RPY angles of the base,
        # the contact implies that in the YawLink frame (after rotation by gamma), we have vx = R (dalpha + dtheta)
        # and vy = 0
        gamma = q[2, 0]
        J = np.matrix([[np.cos(gamma), np.sin(gamma), 0, 0, -self.wheel_radius, -self.wheel_radius],
                       [-np.sin(gamma), np.cos(gamma), 0, 0, 0, 0]])
        dJ = np.matrix([[-np.sin(gamma), np.cos(gamma), 0, 0, 0, 0],
                       [-np.cos(gamma), -np.sin(gamma), 0, 0, 0, 0]]) * v[2, 0]
        drift = -  dJ * v
        
        # Compute controller torque
        torque = np.zeros((self.robot.model.nv, 1))

        torque[5, 0] = self.controller(t, q, v)
        # Write full equation
        A = np.block([[H, J.T], [J, np.zeros((2, 2))]]) 
        b = np.concatenate((torque - g, drift))
        # Find dv, we don't care about the contact forces for now.
        dv = np.linalg.solve(A, b)[:-2]
        
        return np.concatenate((v, dv))
    
    
    def simulate(self, x0, simulation_duration, dt, motor_control_law, output_name = "/tmp/claptrap.csv", verbose = True):
        ''' Run a simulation of given controller motor_control_law, log the results, and return the state.
            @param x0 Initial state (position + velocity)
            @param simulation_duration Length of the simulation
            @param dt Timestep for logger - note that the controller is simulated in a continuous manner
            @param motor_control_law Motor callback law, with signature motor_control_law(t, q, v) -> torque
            @param output_name Optional, name of the output log file.
            @param verbose Optional, whether or not to display a progress bar during simulation.
        '''
        self.controller = motor_control_law
        
        # Create integrator.
        solver = scipy.integrate.ode(self._dynamics)
        solver.set_integrator('dopri5')
        solver.set_initial_value(x0)
        
        # Create logger
        logged_values = ["Claptrap.q" + str(i) for i in range(self.robot.model.nq)] + \
                        ["Claptrap.v" + str(i) for i in range(self.robot.model.nv)] + \
                        ["Claptrap.energy"]
                        
        logger = Logger(logged_values)
        
        if verbose:
            pbar = tqdm(total=simulation_duration, bar_format="{percentage:3.0f}%|{bar}| {n:.2f}/{total_fmt} [{elapsed}<{remaining}]")
        
        t = 0
        result_x = []
        while solver.successful() and t < simulation_duration:
            if verbose:
                pbar.update(dt)
            # Integrate, skip first iteration, we only want to log in this case
            if t > 0:
                solver.integrate(t)
            
            result_x.append(solver.y)
            q = np.matrix(solver.y[:self.robot.nq])
            v = np.matrix(solver.y[self.robot.nq:])
        
            # Log
            for i in range(self.robot.model.nq):
                logger.set("Claptrap.q" + str(i), q[i, 0])
            for i in range(self.robot.model.nv):
                logger.set("Claptrap.v" + str(i), v[i, 0])
            pnc.computeAllTerms(self.robot.model, self.robot.data, q, v)
            energy = self.robot.data.kinetic_energy + self.robot.data.potential_energy
            logger.set("Claptrap.energy", energy)
            logger.set("time", t)
            logger.new_line()
            
            t += dt
            
        
        logger.save(output_name)
        # Return time and x
        return logger.data["time"][:-1], np.array(result_x)
   
    
    # ~ def log_state(self, logger, prefix):
        # ~ '''Log current state: the values logged are defined in CLAPTRAP_STATE_SUFFIXES
        # ~ @param logger Logger object
        # ~ @param prefix Prefix to add before each suffix.
        # ~ '''
        # ~ logger.set(prefix + "roll", self.q[3, 0])
        # ~ logger.set(prefix + "pitch", self.q[4,0])
        # ~ logger.set(prefix + "yaw", self.q[2, 0])
        # ~ # TODO
        # ~ logger.set(prefix + "wheelVelocity", self.v[self.robot.model.joints[self.robot.model.getJointId("WheelJoint")].idx_v, 0])
        
        # ~ pnc.computeAllTerms(self.robot.model, self.robot.data, self.q, self.v)
        # ~ energy = self.robot.data.kinetic_energy + self.robot.data.potential_energy
        # ~ logger.set(prefix + "energy", energy)
        
        # ~ logger.set(prefix + "wheelTorque", self.tau[0, 0])
        
        # ~ w_M_base = self.robot.framePosition(self.q, self.robot.model.getFrameId("Body"), False)
        # ~ logger.set_vector(prefix + "base", w_M_base.translation)
        
