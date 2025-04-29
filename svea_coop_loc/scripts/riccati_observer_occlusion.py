import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt
import time as systemtime
from itertools import combinations

class riccati_observer_occlusion():
    def __init__(self, **kwargs):
        #############
        # self.agent_name = #TODO: some kind of namespace so the vehicle can communicate

        self.ErrMsg = None

        # RK45
        self.tol = kwargs.get("tol", 1e-3)
        self.dt = kwargs.get("dt", 0.0001)
        self.current_time = 0
        self.soly = None
        self.solt = None

        # Parameters for Riccati observer
        self.k = kwargs.get('k', None)
        self.v = kwargs.get('v', None)
        # would be nice to have a differnet coefficient for different groups of agents
        self.q = kwargs.get('q', None)
        self.p_riccati = kwargs.get('p_riccati', None)

        # Variables for Riccati observer
        # orientation and pose estimation
        self.Lambda = kwargs.get('Lambda_hat', None) # quaternion: w, x, y, z
        self.p_hat = kwargs.get('p_hat', None)
        # Velocity measurement
        self.linvel = np.array((3,1)).T
        self.angvel = np.array((3,1)).T
        # Observation
        self.landmarks = {} # id, type, position, bearing
        self.l = 0

        # for updating CRE
        self.V = np.diag(np.hstack([np.diag(self.v[i]*np.eye(3)) for i in range(len(self.v))]))
        self.Q = np.diag(np.hstack([np.diag(self.q*np.eye(3)) for i in range(self.l)])) if self.l != 0 else np.array([])
        self.P_ricatti = np.diag(np.hstack([np.diag(self.p_riccati[i]*np.eye(3)) for i in range(len(self.p_riccati))]))
        
    # Initialization
    def set_init(self, ori, pose):
        self.Lambda = ori
        self.p_hat = pose
        initial_state = np.concatenate((self.Lambda.flatten(), self.p_hat.flatten(), self.P_ricatti.flatten()))
        self.soly = initial_state
        # /riccati_observer_occlusion/set_init
        print(f"""
        ##########################################################################              
            Initial Estimation
            position        | {np.array2string(self.p_hat, separator=', ')}
            orientation     | {np.array2string(self.Lambda, separator=', ')}
        ##########################################################################""")
        

    def update_measurement(self, angular, linear, landmarkInfo, current_time):
        self.linvel = linear
        self.angvel = angular
        self.landmarks = landmarkInfo
        self.current_time = current_time
        self.l = len(self.landmarks)
        if self.l == 0:
            self.Q = []
        else:
            self.Q = np.diag(np.hstack([np.diag(self.q*np.eye(3)) for i in range(self.l)])) if self.l != 0 else np.array([])

    # TODO: doesnt it make more sense to do cross product??
    def function_S(self, input):
        '''
        Create a 3x3 skew-symmetric matrix, S(x)y = x x y
        Input: 3x1 array
        Output: 3x3 array
        '''
        # input should be array
        # output array
        flattened_input = input.flatten()
        output = [[0,           -flattened_input[2],    flattened_input[1]],
                [flattened_input[2],  0,              -flattened_input[0]],
                [-flattened_input[1], flattened_input[0],     0]]
        return np.array(output)

    def rodrigues_formula(self, quaternion):
        '''
        Quaternion -> R_tilde_bar
        Input: [w,x,y,z]
        Output R_tile_bar (rotation matrix)
        From page6
        '''
        return np.eye(3) + 2*np.matmul(self.function_S(quaternion[1:]), (quaternion[0]*np.eye(3) + self.function_S(quaternion[1:])))
    
    def function_Pi(self, input):
        '''
        Pi_x := I_3 - xx^T
        Input: array
        Output P_x
        '''
        return np.eye(3) - np.outer(input, input)
   
    def function_C(self, input_R_hat_i, input_hat_p_i):
        '''
        Create the C maxtrix 
        Input = ...
        Output = num_landmark*3x6 matrix
        '''
        j = 0
        for landmark_id, landmark_info in self.landmarks.items():
            # pi_g -- Bearing is the actual measurement
            pi_g = self.function_Pi(landmark_info["bearing"])
            # Pi(g_ij) hat_R_i^T
            second = np.matmul(pi_g, np.transpose(input_R_hat_i))
            # epsolon = R_hat_^T (p_hat - z)
            # Pi(g_ij) S(epsolon) hat_R_i^T
            # original paper (first)
            # first = np.matmul(pi_g, np.matmul(function_S(np.matmul(np.transpose(input_R_hat_i), input_p_hat_ij[j])), np.transpose(input_R_hat_i)))
            # distributed pose localization (first)
            first = np.matmul(pi_g, np.matmul(np.transpose(input_R_hat_i), self.function_S((input_hat_p_i - landmark_info["position"]))))
            C_landmark = np.hstack((first, second))
            if j == 0:
                output_C = C_landmark
            else:
                output_C = np.vstack((output_C, C_landmark))
            j += 1

        return output_C
    
    def observer_equations(self, input_R_hat_i, input_p_hat_i, input_P):
        first_upper = self.angvel
        first_lower = self.linvel
        first_part = np.hstack((first_upper, first_lower)).reshape((6,1))
        # second
        final = np.transpose(np.array([0, 0, 0], dtype=np.float64))
        final2 = np.transpose(np.array([0, 0, 0], dtype=np.float64))
        self.current_correction = []

        for landmark_id, landmark_info in self.landmarks.items():
            # pi_g_ij
            pi_g = self.function_Pi(landmark_info["bearing"])
            # pi_g_ij * R_hat_i.T * (p_hat_i - z_j)
            common = np.matmul(pi_g, np.matmul(np.transpose(input_R_hat_i), (input_p_hat_i- landmark_info["position"])))
            # S(R_hat_i.T * (p_hat_i - z_j))
            S_rp = self.function_S(np.matmul(np.transpose(input_R_hat_i),  (input_p_hat_i - landmark_info["position"])))
            # q* R_hat_i * S^T() * pi_g * R_hat_i.T * p_hat_ij
            upper = self.q * np.matmul(input_R_hat_i, np.matmul(np.transpose(S_rp), common))
            # q* R_hat_i * pi_g * R_hat_i.T * p_hat_ij
            lower = self.q * np.matmul(input_R_hat_i, common)
            self.current_correction.append(np.concatenate([upper,lower]).tolist())
            final += upper
            final2 += lower
        second_part = np.vstack((final, final2)).reshape((6,1))

        RI_matrixupper = np.hstack((np.transpose(input_R_hat_i), np.zeros((3,3))))
        RI_matrixlower = np.hstack((np.zeros((3,3)), np.eye(3)))
        RI_matrix = np.vstack((RI_matrixupper, RI_matrixlower))
        second_part = self.k * np.matmul(RI_matrix, np.matmul(input_P, second_part))
        output_omega_hat_p_hat_dot = first_part - second_part
        return output_omega_hat_p_hat_dot
    
    def dynamics(self, t, y):
        # , input_k, input_z, input_q, input_Q, input_V, numNeighbor, is_z_moving, p_formula, omega_formula, v_formula
        qua_hat_flat, p_hat_flat, input_P_flat = np.split(y, [4, 7])
        qua_hat_flat = qua_hat_flat/np.linalg.norm(qua_hat_flat)
        input_R_hat = self.rodrigues_formula(qua_hat_flat)

        input_p_hat = p_hat_flat #.reshape((3,1))
        input_P = input_P_flat.reshape((6,6))

        if self.l != 0:
            input_C = self.function_C(input_R_hat, input_p_hat)
        ####################################

        ####################################
        ############# Observer #############
        output_omega_hat_p_hat_dot = self.observer_equations(input_R_hat, input_p_hat, input_P)
        ############# Observer #############
        ####################################

        if self.l != 0:
            output_P_dot = - np.matmul(input_P, np.matmul(np.transpose(input_C), np.matmul(self.Q, np.matmul(input_C, input_P)))) + self.V
        else:
            output_P_dot = self.V
            
        omega_hat = output_omega_hat_p_hat_dot[0:3]
        omega_hat = omega_hat.reshape((-1,))
        p_hat_dot = output_omega_hat_p_hat_dot[3:]
        ####################################
        ############ Quaternion ############
        omega_hat_4x4 = np.array([[0, -omega_hat[0], -omega_hat[1], -omega_hat[2]],
                                [omega_hat[0], 0, omega_hat[2], -omega_hat[1]],
                                [omega_hat[1], -omega_hat[2], 0, omega_hat[0]],
                                [omega_hat[2], omega_hat[1], -omega_hat[0], 0]])

        output_qua_hat_flat = 0.5*np.matmul(omega_hat_4x4, qua_hat_flat)

        return np.concatenate((output_qua_hat_flat, p_hat_dot.flatten(), output_P_dot.flatten()))
        ########### Quaternion ############
        ####################################

    def rk45_step(self):
        # , t, y, dt, args, tol
        a2, a3, a4, a5, a6 = 1/5, 3/10, 4/5, 8/9, 1
        b21 = 1/5
        b31, b32 = 3/40, 9/40
        b41, b42, b43 = 44/45, -56/15, 32/9
        b51, b52, b53, b54 = 19372/6561, -25360/2187, 64448/6561, -212/729
        b61, b62, b63, b64, b65 = 9017/3168, -355/33, 46732/5247, 49/176, -5103/18656
        c1, c2, c3, c4, c5, c6 = 35/384, 0, 500/1113, 125/192, -2187/6784, 11/84

        c1_4, c3_4, c4_4, c5_4, c6_4 = 5179/57600, 7571/16695, 393/640, -92097/339200, 187/2100
        # Runge-Kutta stages
        k1 = self.dynamics(self.current_time, self.soly)
        k2 = self.dynamics(self.current_time + a2*self.dt, self.soly + self.dt*b21*k1)
        k3 = self.dynamics(self.current_time + a3*self.dt, self.soly + self.dt*(b31*k1 + b32*k2))
        k4 = self.dynamics(self.current_time + a4*self.dt, self.soly + self.dt*(b41*k1 + b42*k2 + b43*k3))
        k5 = self.dynamics(self.current_time + a5*self.dt, self.soly + self.dt*(b51*k1 + b52*k2 + b53*k3 + b54*k4))
        k6 = self.dynamics(self.current_time + a6*self.dt, self.soly + self.dt*(b61*k1 + b62*k2 + b63*k3 + b64*k4 + b65*k5))

        # Update step
        y_next = self.soly + self.dt*(c1*k1 + c2*k2 + c3*k3 + c4*k4 + c5*k5 + c6*k6)

        y_next_4 = self.soly + self.dt * (c1_4*k1 + c3_4*k3 + c4_4*k4 + c5_4*k5 + c6_4*k6)

        error = np.abs(y_next - y_next_4)
        error_norm = np.linalg.norm(error)
        safety_factor = 0.9
        min_scale_factor = 0.2
        max_scale_factor = 40.0
        if error_norm <= self.tol:
            success = True
            t = self.current_time + self.dt
            dt = self.dt * min(max_scale_factor, max(min_scale_factor, safety_factor * (self.tol / error_norm)**0.25))
        else:
            success = False
            t = self.current_time
            dt = self.dt * max(min_scale_factor, safety_factor * (self.tol / error_norm)**0.25)
        return y_next, t, dt, success
    
    def solver(self):
        #k, z, q, Q, V, l, is_z_moving, p_formula, omega_formula, v_formula, tol, dt, time, soly
        ### Run solver
        ######################################################
        ####################### Solver #######################
        success = False
        while not success:
            y_next, next_time, new_dt, success = self.rk45_step()
            if success:
                self.soly = y_next
                self.solt = next_time
            self.dt = new_dt

        return (self.solt, self.dt, self.soly, self.ErrMsg)
        ####################### Solver #######################
        ######################################################