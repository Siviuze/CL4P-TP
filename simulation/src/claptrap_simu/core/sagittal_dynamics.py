# Explicit analytical equations of the dynamics of a Segway (i.e. a simple Claptrap, with no upper body)
# in the sagittal plane.
# The non-linear version is meant to be used as simulation validation, while a linearized version is made availble
# for study and gain tuning (for instance, using LQR).
# For more information on the equations displayed here, have a look at ControlDocumentation.pdf

import numpy as np
import scipy.integrate
import pkg_resources
import matplotlib.pyplot as plt

import pinocchio as pnc


class ClaptrapSagittalDynamics():
    '''  Exact and linearized dynamics of a Segway in the sagittal plane
    '''
    def __init__(self, urdf_path=None):
        ''' Constructor
        '''
        # Get parameters from the URDF.
        if urdf_path is None:
            urdf_path = pkg_resources.resource_filename('claptrap_simu', 'data/claptrap.urdf')
        model = pnc.buildModelFromUrdf(urdf_path)
        self.g = 9.81 # Gravity
        self.m = model.inertias[model.getJointId("BodyJoint")].mass # Mass of the body
        self.mw = model.inertias[model.getJointId("WheelJoint")].mass # Mass of the wheel
        self.I =  model.inertias[model.getJointId("BodyJoint")].inertia[1,1] # Body inertia at CoM arong Y
        self.J =  model.inertias[model.getJointId("WheelJoint")].inertia[1,1] # Wheel inertia at CoM around Y

        self.l = model.inertias[model.getJointId("BodyJoint")].lever[2] # Distance of CoM of body from wheel axis
        self.r = model.jointPlacements[model.getJointId("BodyJoint")].translation[2] # Radius of the wheel

        # Define some constants that appear often in the inertia matrix
        self.mlr= self.m * self.l * self.r
        self.Mr2 = (self.m + self.mw) * self.r**2

        # Motor control law
        self.motor_control_law = None

    def compute_inertia_matrix(self, theta):
        ''' Compute inertia matrix from current base angle
        '''
        H = np.array([[2 * self.mlr * np.cos(theta) + self.m * self.l**2 + self.Mr2 + self.I + self.J,  self.mlr * np.cos(theta) + self.Mr2 + self.J],
                       [self.mlr * np.cos(theta) + self.Mr2 + self.J, self.Mr2 + self.J]])
        return H

    def nonlinear_dynamics(self, t, x):
        ''' Nonlinear dynamics of claptrap, meant to be used by an ODE solver.
        @param t Current time
        @param x Current state, (theta, phi, dtheta, dphi, x). Theta is the base angle wrt the world, phi the relative
                 wheel displacement (i.e. encoder reading).
        @param Callback torque function
        '''

        theta = x[0]
        phi = x[1]
        dtheta = x[2]
        dphi = x[3]

        # Compute inertia matrix
        H = self.compute_inertia_matrix(theta)
        # Comptue nonlinear effects
        nle = np.array([(self.m * self.l * self.g  + self.mlr * dtheta**2)* np.sin(theta),
                        self.mlr * dtheta**2 * np.sin(theta)])

        tau = np.zeros(2)
        if self.motor_control_law is not None:
            tau[1] = self.motor_control_law(t, x)

        # Solve for acceleration
        ddq = np.linalg.solve(H, nle + tau)

        dx = np.array([dtheta, dphi, ddq[0], ddq[1], self.r * (dtheta + dphi)])
        return dx

    def run_nonlinear_simulation(self, x0, dt, simulation_duration, motor_control_law = None):
        ''' Run a simulation with the nonlinear equation.
        @param x0 Initial point.
        @param dt Sampling period of output.
        @param simulation_duration Duration to simulate
        @param motor_control_law Optional, callback with signature motor_control_law(t, x) -> float 
                                 or a np.array for linear controller (in which case the callback is - K x)
        '''
        if isinstance(motor_control_law, np.ndarray):
            def motor_callback(t, x):
                return -(motor_control_law @ x[:4])[0]
            self.motor_control_law = motor_callback
        else:
            self.motor_control_law = motor_control_law
        
        if len(x0) == 4:
            # Add x coordinate if not present
            x0 = np.concatenate((x0 , np.zeros(1)))

        solver = scipy.integrate.ode(self.nonlinear_dynamics)
        solver.set_integrator('dopri5')
        solver.set_initial_value(x0)

        time = [0.0]
        x = [x0]
        torque = [0]
        t = dt
        while solver.successful() and t < simulation_duration:
            # Integrate for dt
            solver.integrate(t)
            # Save result
            time.append(t)
            x.append(solver.y)
            torque.append(self.motor_control_law(t, solver.y))
            t += dt

        return time, x, torque
    
    def get_linear_system_matrices(self):
        ''' 
        @brief Return A, B, the linear matrices linked to this system (for simulation or gain tuning).
        '''
        H_inv = np.linalg.inv(self.compute_inertia_matrix(0))
        A = np.array([[0,                                     0, 1, 0],
                      [0,                                     0, 0, 1],
                      [H_inv[0,0] * self.m * self.l * self.g, 0, 0, 0],
                      [H_inv[1 ,0] * self.m * self.l * self.g, 0, 0, 0]])
        B = np.array([[0],
                      [0],
                      [H_inv[0, 1]],
                      [H_inv[1,1]]])
        return A, B
    
    def run_linear_simulation(self, x0, dt, simulation_duration, K):
        ''' Run a simulation with the linear model, and a linear controller.
        @param x0 Initial point.
        @param dt Sampling period of output.
        @param simulation_duration Duration to simulate
        @param K Controller gain (row matrix)
        '''
        
        # Compute close-loop matrix
        A, B = self.get_linear_system_matrices()
        M = A - B @ K
        
        # Compue analytical solution
        time = np.arange(0, simulation_duration, dt)
        x = [scipy.linalg.expm(M * t) @ x0 for t in time]
        
        return time, x, [- (K @ v)[0] for v in x]
    
    def plot(self, solutions, legends):
        ''' Plot the results of several simulations on the same figure.
        @param times List of timestamps for each simulation.
        @param solutions List of tuples (time, state, torque) returned by the various itne List of states for each simulation.
        @param torques List of motor torque for each simulation.
        @param legends Legend.
        '''
        # Extract time, state, command from input tuple
        times = [s[0] for s in solutions]
        states = [np.array(s[1]) for s in solutions]
        torques = [s[2] for s in solutions]
        
        ax1 = plt.subplot(221)
        plt.title("Body angle")
        for i in range(len(times)):
            plt.plot(times[i], states[i][:, 0], label = legends[i])
        plt.legend()
        plt.grid()
        
        plt.subplot(222, sharex = ax1)
        plt.title("Total wheel rotation")
        for i in range(len(times)):
            plt.plot(times[i], states[i][:, 0] + states[i][:, 1], label = legends[i])
        plt.legend()
        plt.grid()
        
        plt.subplot(223, sharex = ax1)
        plt.title("Body velocity")
        for i in range(len(times)):
            plt.plot(times[i], states[i][:, 2], label = legends[i])
        plt.legend()
        plt.grid()
        
        plt.subplot(224, sharex = ax1)
        plt.title("Torque")
        for i in range(len(times)):
            plt.plot(times[i], torques[i], label = legends[i])
        plt.legend()
        plt.grid()
        plt.show()
