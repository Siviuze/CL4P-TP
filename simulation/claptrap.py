# Claptrap class
# This class runs a simulation of Claptrap using pinocchio, given a controller as input.

# A controller is defined by the following signature:
# def controller(t, q, v, q_ref, v_ref, a_ref):
#   return tau


import pinocchio as se3
import meshcat
import numpy as np
import scipy.integrate

# Suffix used when logging the state
CLAPTRAP_STATE_SUFFIXES = ["roll", "pitch", "yaw"] + \
                          ["baseX", "baseY", "baseZ"] + \
                          ["omegaX", "omegaY", "omegaZ"] + \
                          ["wheelVelocity", "energy", "wheelTorque"]

def skew_symmetric(v):
    return np.matrix([[0, -v[2,0], v[1,0]], [v[2,0], 0, -v[0, 0]], [-v[1, 0], v[0, 0], 0]])

class Claptrap():
    def __init__(self, controller, initial_state = None, meshcat_viewer = None, meshcat_name = "claptrap", robot_color = None):
        '''
            Init the robot object.
            @param controller Controller callback.
            @param initial_state Optional, initial state as tuple (q0, v0) to start the simulation (position and velocity)
            @param meshcat_viewer Optional, meshcat viewer. If set, this viewer can be used to display the robots during simulation.
            @param meshcat_name Optional, if meshcat_viewer is set, name of the robot in the viewer.
            @param robot_color Optional, if meshcat_viewer is set, color of the robot in the viewer.
        '''
        self.meshcat_viewer = meshcat_viewer 
        self.controller = controller
        
        self.robot = se3.RobotWrapper.BuildFromURDF("urdf/claptrap.urdf", ["urdf/"], root_joint=se3.JointModelFreeFlyer())
        
        # Compute wheel radius vector.
        self.wheel_radius = self.robot.model.frames[self.robot.model.getFrameId("RimJoint")].placement.translation
        
        # Current robot state
        if initial_state is not None:
            self.q = initial_state[0]
            self.v = initial_state[1]
        else:
            self.q = self.robot.q0
            self.q[:3, 0] = self.wheel_radius
            self.v = np.matrix(np.zeros((self.robot.nv, 1)))
            
        # Initialize viewer if needed.
        if self.meshcat_viewer is not None:
            self.robot.initMeshcatDisplay(self.meshcat_viewer, meshcat_name, robot_color)
            self.robot.display(self.robot.q0)
        
        # Create integrator.
        self.solver = scipy.integrate.ode(self.dynamics)
        self.solver.set_integrator('dopri5')
        self.solver.set_initial_value(np.concatenate((self.q, self.v)))
    
    
    def integrate(self, t):
        ''' Integrate simulation up to time t'''
        # Run integration
        self.solver.integrate(t)
        # Extract data from solver
        self.q = np.matrix(self.solver.y[:self.robot.nq])
        # Renormalize quaternion to prevent propagation of rounding errors due to integration.
        self.q[3:7, 0] /= np.linalg.norm(self.q[3:7, 0])
        self.v = np.matrix(self.solver.y[self.robot.nq:])
        self.tau = self.controller(t, self.q, self.v, None, None, None)
        return self.solver.successful()
    
    
    def dynamics(self, t, x):
        ''' Forward dynamics of the robot, to integrate
        '''
        # Split input as (q, v) pair
        q = np.matrix(x[:self.robot.nq]).T
        v = np.matrix(x[self.robot.nq:]).T
        # Renormalize quaternion to prevent propagation of rounding errors due to integration.
        q[3:7, 0] /= np.linalg.norm(q[3:7, 0])
        
        # Run forward dynamic computation
        # Compute H and g + coloriolis effects
        H =  self.robot.mass(q)
        g = self.robot.nle(q, v)
        
        # Compute jacobian
        # Note that the jacobian only depends on data directly available in q, v, so there is no need for advanced
        # computation
        radius_world =  se3.Quaternion(q[6, 0], q[3, 0], q[4, 0], q[5, 0]).conjugate() * self.wheel_radius
        omega = v[3:6, 0]
        skew_radius = skew_symmetric(radius_world)
        omega_skew_radius = skew_symmetric(- skew_symmetric(omega) * radius_world)
        
        J = np.concatenate((np.eye(3), skew_radius, skew_radius * np.matrix([[0, 1, 0]]).T), axis=1)
        dJ = np.concatenate((np.zeros((3,3)), omega_skew_radius, omega_skew_radius * np.matrix([[0, 1, 0]]).T), axis=1)
        drift = -dJ * v
        
        # Compute controller torque
        torque = np.zeros((self.robot.model.nv, 1))
        # Consider all joints actuated except freeflyer
        torque[6:, 0] = self.controller(t, q, v, None, None, None)
        # Write full equation
        A = np.block([[H, J.T], [J, np.zeros((3,3))]]) 
        # ~ b = np.concatenate((torque - g, drift))
        b = np.concatenate((torque - g, drift))
        
        # Find dv
        dv = np.linalg.solve(A, b)[:-3]
        
        # Now we just need to integrate dv
        # The slight issue is that q is not a term-by-term integral of v (because of the freeflyer).
        # Here we use pinocchio's integrate funciton to compute the numerical derivative of q
        dt = 1e-6
        dq = ((se3.integrate(self.robot.model, q, dt * v) - q)/ dt)
        
        return np.concatenate((dq, dv))
    
    def log_state(self, logger, prefix):
        '''Log current state: the values logged are defined in CLAPTRAP_STATE_SUFFIXES
        @param logger Logger object
        @param prefix Prefix to add before each suffix.
        '''
        rpy = se3.rpy.matrixToRpy(se3.Quaternion(self.q[6, 0], self.q[3, 0], self.q[4, 0], self.q[5, 0]).matrix())
        logger.set(prefix + "roll", rpy[0,0])
        logger.set(prefix + "pitch", rpy[1,0])
        logger.set(prefix + "yaw", rpy[2,0])
        logger.set_vector(prefix + "omega", self.v[3:6, 0])
        logger.set(prefix + "wheelVelocity", self.v[self.robot.model.joints[self.robot.model.getJointId("WheelJoint")].idx_v, 0])
        
        se3.computeAllTerms(self.robot.model, self.robot.data, self.q, self.v)
        energy = self.robot.data.kinetic_energy + self.robot.data.potential_energy
        logger.set(prefix + "energy", energy)
        
        logger.set(prefix + "wheelTorque", self.tau[0, 0])
        
        w_M_base = self.robot.framePosition(self.q, self.robot.model.getFrameId("Body"), False)
        logger.set_vector(prefix + "base", w_M_base.translation)
        
