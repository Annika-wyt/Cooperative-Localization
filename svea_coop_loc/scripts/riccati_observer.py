import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt
import time as systemtime
from itertools import combinations

ERROR_MSG = {-1:"No error",
             0: "Not enough source points",
             1: "Algined source points",
             2: "Three non-aligned source points: moving along one of the straight lines of the danger cylinder and passing through a source point",
             3: "Three non-aligned source points: motionless C in danger cylinder",
             4: "Three non-aligned source points: moving on danger cylinder but not along any lines (weak)",
             5: "Four + non-aligned source points: on horopter curve",
            }

class riccati_observer():
    def __init__(self, **kwargs):
        ######################################################
        ##################### Parameters #####################
        self.z_groundTruth = np.array([])
        self.z = np.array([])

        self.which_eq = kwargs.get('which_eq', 0)
        self.stepsize = kwargs.get('stepsize', 0.1)
        self.tol = kwargs.get('tol', 1e-2 * 3) 

        self.soly = None
        self.solt = None
        ##################### Parameters #####################
        ######################################################

        ######################################################
        ################### initialization ###################
        # landmarks
        self.z = kwargs.get('z', np.array([])) # in svea frame
        self.l = len(self.z)
        
        self.k = kwargs.get('k', 1)
        self.v = kwargs.get('v', [0.1,1])
        self.q = kwargs.get('q', 10)
        self.V = np.diag(np.hstack([np.diag(self.v[i]*np.eye(3)) for i in range(len(self.v))]))
        self.Q = np.diag(np.hstack([np.diag(self.q*np.eye(3)) for i in range(self.l)])) if self.l != 0 else np.array([])
        
        self.p_ricatti = kwargs.get('p_ricatti', [1.0,100.0])
        self.P_ricatti = np.diag(np.hstack([np.diag(self.p_ricatti[i]*np.eye(3)) for i in range(len(self.p_ricatti))]))

        self.Lambda_bar_0 = kwargs.get('Lambda_bar_0', np.array([1, 0, 0, 0], dtype=np.float64).T)  # quaternion: w, x, y, z
        self.Rot_hat = kwargs.get('Rot_hat', self.rodrigues_formula(self.Lambda_bar_0))
        self.p_hat = kwargs.get('p_hat', np.array([[0, 0, 0]], dtype=np.float64).T)
        self.p_bar_hat = self.add_bar(self.Rot_hat, self.p_hat)

        self.linearVelocity = None
        self.angularVelocity = None

        self.direction = []
        self.ErrMsg = None
        
        self.current_time = 0
        self.running_rk45 = False

        self.dt = self.stepsize

        self.soly = np.concatenate((self.Lambda_bar_0.flatten(), self.p_bar_hat.flatten(), self.P_ricatti.flatten()))

        ################### initialization ###################
        ######################################################
        # self.print_init()

    def set_init(self, ori, pose):
        self.Lambda_bar_0 = ori
        self.p_hat = pose
        self.Rot_hat = self.rodrigues_formula(self.Lambda_bar_0)
        self.p_bar_hat = self.add_bar(self.Rot_hat, self.p_hat)
        self.soly = np.concatenate((self.Lambda_bar_0.flatten(), self.p_bar_hat.flatten(), self.P_ricatti.flatten()))
        print(f"""/riccati_observer/set_init
##########################################################################              
    Initial Estimation
    position        | {np.array2string(self.p_bar_hat, separator=', ')}
    orientation     | {np.array2string(self.Rot_hat[0].flatten(), separator=', ')}
                    | {np.array2string(self.Rot_hat[1].flatten(), separator=', ')}
                    | {np.array2string(self.Rot_hat[2].flatten(), separator=', ')}

##########################################################################""")

    def print_init(self):
        Q_str = '   \n'.join(['                             ' + '  '.join(map(str, row)) for row in self.Q])
        V_str = '   \n'.join(['                             ' + '  '.join(map(str, row)) for row in self.V])
        P_ricatti_str = '   \n'.join(['                             ' + '  '.join(map(str, row)) for row in self.P_ricatti])

        print(f"""
        Parameters
        stepsize               | {self.stepsize}
        tol                    | {self.tol}
        which_eq               | {self.which_eq}
        k                      | {self.k}
        Q                      |
    {Q_str}
        V                      |
    {V_str}
        P ricatti              |
    {P_ricatti_str}
        """)

    def update_measurement(self, angular, linear, landmark, landmarkGroundTruth, current_time):
        direction = self.calculate_direction(landmark)
        # success, ErrMsg = self.checkObservability(direction, angular, linear, landmark, landmarkGroundTruth)
        self.angularVelocity = angular
        self.linearVelocity = linear
        if len(landmark) > 2:
            success = True
            self.ErrMsg = -1
        else:
            success = False
            self.ErrMsg = 0

        if success:
            self.direction = direction
            self.current_time = current_time
            self.z = landmark
            self.l = len(self.z)
            if len(landmark) == 0:
                self.Q = []
            else:
                self.Q = np.diag(np.hstack([np.diag(self.q*np.eye(3)) for i in range(self.l)])) if self.l != 0 else np.array([])
            self.z_groundTruth = landmarkGroundTruth
        else:
            self.direction = []
            self.current_time = current_time
            self.z = []
            self.l = 0
            self.Q = []
            self.z_groundTruth = []
        return success
            
    def checkObservability(self, dirs, angular, linear, lm, lmGt):
        if len(lm) >= 3:
            # check if align
            aligned = self.checkAlignedPoints(lmGt)
            if aligned:
                return (False, 1)
            motionless = self.checkMotionless(angular, linear)
            if len(lm) > 3 and motionless:
                #TODO: horopter
                pass
            if len(lm) == 3:
                motion_less_danger_cylinder = self.checkDangerCylinder(lm, lmGt)
                if motion_less_danger_cylinder:
                    return (False, 3)
                if not motionless:
                    moving_towards_point = self.checkMotion(angular, linear, lmGt)
                    if moving_towards_point:
                        return (False, 2)
            return (True, -1)
        return (False, 0)
    
    def checkMotion(self, angular, linear, lmGt):
        ori = self.soly[0:4]
        ori /= np.linalg.norm(ori)
        rot = self.rodrigues_formula(ori)
        vel_F = self.remove_bar(rot, linear)
        vel_F /= np.linalg.norm(vel_F)
        base_vec = np.subtract(lmGt[2], lmGt[0])
        base_vec /= np.linalg.norm(base_vec)
        if np.allclose(np.dot(vel_F , base_vec), np.zeros(1), atol=0.0001):
            pose = np.matmul(self.rodrigues_formula(self.soly[0:4]), self.soly[4:7])
            for pt in lmGt:
                vec = np.subtract(pose, pt)
                vec /= np.linalg.norm(vec)
                if np.allclose(np.dot(vec , base_vec), np.zeros(1), atol=0.0001):
                    return True
        return False
        
    def checkAlignedPoints(self, lmGt):
        # just to simplify stuff, if there are algined points and the remaining points
        # are not at least 5 meters away (from the closet point on the line), the measurements
        # for the markers will not be used

        # check in simulation as well to see if we get the same behavior
        alignedCounter = 0
        for group in combinations(lmGt, 3):
            p1, p2, p3 = group
            vec1 = np.subtract(p2, p1)
            vec2 = np.subtract(p3, p1)
            if np.allclose(np.cross(vec1, vec2), np.zeros(3), atol=np.ones(3)*0.01): 
                alignedCounter += 1
        if len(lmGt) <=5 and alignedCounter != 0:
            return True
        elif len(lmGt) > 5 and alignedCounter > 1:
            return True
        else:
            return False
        
    def checkMotionless(self, angular, linear):
        if np.allclose(linear, np.zeros((3,1)), atol=1e-7):
            return True
        return False
    
    def checkDangerCylinder(self, lm, lmGt):
        if len(lmGt) == 3:
            a = np.transpose(self.function_S(np.subtract(lmGt[1],lmGt[0])))
            b = np.subtract(lmGt[0],lmGt[1]).reshape((3,1))
            c = np.array(lm[1]).reshape((3,1))
            # c = np.subtract(pose, lmGt[1]).reshape((3,1))
            upper = np.hstack((a,b,c,np.zeros((3,1))))

            a = np.transpose(self.function_S(np.subtract(lmGt[2],lmGt[0])))
            b = np.subtract(lmGt[0],lmGt[2]).reshape((3,1))
            c = np.array(lm[2]).reshape((3,1))
            # c = np.subtract(pose, lmGt[2]).reshape((3,1))
            lower = np.hstack((a,b,np.zeros((3,1)),c))
            combined = np.vstack((upper, lower))
            det = np.linalg.det(combined)
            if det == 0:
                return True
            return False
        else:
            return True
    
    def function_S(self, input):
        '''
        Create a 3x3 skew-symmetric matrix, S(x)y = x x y
        Input: 3x1 array
        Output: 3x3 array
        '''
        # input should be array
        # output array
        output = [[0,           -input[2],    input[1]],
                [input[2],  0,              -input[0]],
                [-input[1], input[0],     0]]
        return np.array(output)

    def rodrigues_formula(self, quaternion):
        '''
        Quaternion -> R_tilde_bar
        Input: [w,x,y,z]
        Output R_tile_bar (rotation matrix)
        From page6
        '''
        return np.eye(3) + 2*np.matmul(self.function_S(quaternion[1:]), (quaternion[0]*np.eye(3) + self.function_S(quaternion[1:])))

    def function_A(self):
        '''
        Create the A maxtrix 
        Input = 3x1 array
        Output = 6x6 matrix
        '''
        A11 = self.function_S(-self.angularVelocity)
        A12 = np.zeros((3,3))
        A21 = np.zeros((3,3))
        A22 = self.function_S(-self.angularVelocity)
        return np.vstack((np.hstack((A11, A12)), np.hstack((A21, A22))))

    def function_Pi(self, input):
        '''
        Pi_x := I_3 - xx^T
        Input: array
        Output P_x
        '''
        return np.eye(3) - np.outer(input, input)

    # def function_d(self, input_rot, input_p, input_z):
    #     '''
    #     Calculate direction d_i(t) := R^T(t)(p(t) - z_i)/|p(t)-z_i|
    #     Input:
    #         Rotation matrix R: 3x3 array
    #         pose p: 3x1 array
    #         landmark z : 3x1 array
    #         with_noise : boolean
    #     Output: 
    #         direction vector 3x1 array
    #     '''
    #     norm = (input_p - input_z)/np.linalg.norm(input_p - input_z)
    #     dir = np.matmul(np.transpose(input_rot), norm)
    #     return dir

    def function_C(self, input_R_hat):
        '''
        Create the C maxtrix 
        Input = ...
        Output = num_landmark*3x6 matrix
        '''
        if self.l != 0:
            try:
                for landmark_idx in range(self.l):
                    d = self.direction[landmark_idx]
                    first = self.function_Pi(d)
                    second = self.function_S(np.matmul(np.transpose(input_R_hat), np.array(self.z_groundTruth[landmark_idx]))) # self.function_S(np.matmul(np.transpose(input_R_hat), self.z[landmark_idx])) #TODO
                    final = -np.matmul(first, second)
                    C_landmark = np.hstack((final, first))
                    if landmark_idx == 0:
                        output_C = C_landmark
                    else:
                        output_C = np.vstack((output_C, C_landmark))
            except Exception as e:
                print(f"OPS, function C {e}")
                output_C = []
                self.l = 0
                self.z = []
                self.z_groundTruth = []
        else: 
            output_C = []
        return output_C

    def add_bar(self, input_rot, input_p):
        '''
        Change frame (F -> B)
        '''
        return np.matmul(np.transpose(input_rot), input_p)
    
    def remove_bar(self, input_rot, input_p_bar):
        '''
        Change frame (B -> F)
        '''
        return np.matmul(np.linalg.inv(np.transpose(input_rot)), input_p_bar)
    
    def calculate_direction(self, landmark):
        direction = []
        for landmark_idx in range(len(landmark)):
            a = np.array(landmark[landmark_idx]/ np.linalg.norm(landmark[landmark_idx]))
            # a = -a
            direction.append(a)
        return direction

    def observer_equations(self, input_p_bar_hat, input_R_hat, input_P):
        if self.which_eq == 0:
            # omega
            first_upper = self.angularVelocity
            # first_upper = self.add_bar(input_R_hat, self.angularVelocity)
            
            # -S(omega)p_bat_hat + v_bar
            first_lower = -np.cross(self.angularVelocity, input_p_bar_hat) + self.linearVelocity

            first_part = np.hstack((first_upper, first_lower))
            # omega_hat second part upper
            if len(self.z) != 0:
                final = np.array([0, 0, 0], dtype=np.float64)
                final2 = np.array([0, 0, 0], dtype=np.float64)

                for landmark_idx in range(self.l):
                    #R_hat.T z #TODO: huh??? different from original
                    first = np.matmul(np.transpose(input_R_hat), self.z_groundTruth[landmark_idx])
                    # first = np.matmul(np.transpose(input_R_hat), np.array(landmark[landmark_idx]))
                    #Pi_d
                    d = self.direction[landmark_idx]
                    # d = -d
                    Pi_d = self.function_Pi(d)
                    #(p_bar_hat - R_hat.T x z)
                    second = input_p_bar_hat - first
                    # q*
                    final += self.q*np.matmul(np.transpose(np.cross(first, Pi_d)), second)
                    # omega_hat second part lower
                    #q*Pi_d
                    #(p_bar_hat - R_hat.T x z)
                    final2 += self.q*np.matmul(Pi_d, second)

                second_part = np.hstack((final, final2))

                #kP[]
                #full second part 
                second_part = self.k*np.matmul(input_P, second_part)
                # print("second part", second_part)
                # Final
                output_omega_hat_p_bar_hat_dot = first_part - second_part
            else:
                output_omega_hat_p_bar_hat_dot = first_part

        elif self.which_eq == 1:
            pass
            # print("NO EQUATION 1")

        elif self.which_eq == 2:
            ### First part ###
            # omega hat
            first_upper = self.angularVelocity

            # -S(w)p_bar_hat + v_bar
            first_lower = -np.cross(self.angularVelocity, input_p_bar_hat) + self.linearVelocity
            # first part final
            first_part = np.hstack((first_upper, first_lower))

            if len(self.z) != 0:
                ### Second part ###
                final = np.transpose(np.array([0, 0, 0], dtype=np.float64))
                final2 = np.transpose(np.array([0, 0, 0], dtype=np.float64))
                for landmark_idx in range(self.l):
                    d = np.array(self.z[landmark_idx]/ np.linalg.norm(self.z[landmark_idx]))
                    d_bar_hat = np.array(self.z_groundTruth[landmark_idx])/np.linalg.norm(self.z_groundTruth[landmark_idx])
                    Pi_d_bar_hat = self.function_Pi(d_bar_hat)
                    # S(R_hat.T z) Pi_d_bar_hat 
                    # first = np.cross(np.array(self.z_groundTruth[landmark_idx]), Pi_d_bar_hat)
                    first = np.matmul(self.function_S(np.array(self.z_groundTruth[landmark_idx])), Pi_d_bar_hat)
                    # |p_bar_hat - R_hat.T z| di
                    second = (np.linalg.norm(self.z_groundTruth[landmark_idx])*d).reshape((3,1))
                    final += np.matmul(first, second).reshape((3,))

                    # Pi_d_bar_hat
                    first = Pi_d_bar_hat
                    # |p_bar_hat - R_hat.T z| di
                    #second 
                    final2 += np.matmul(first, second).reshape((3,))

                second_part = np.hstack((final, final2))
                second_part = self.k*self.q*np.matmul(input_P, second_part)

                output_omega_hat_p_bar_hat_dot = first_part + second_part
            else:
                output_omega_hat_p_bar_hat_dot = first_part
        return output_omega_hat_p_bar_hat_dot

    def dynamics(self, t, y):
        # pose
        ####################################
        ############ Quaternion ############
        qua_hat_flat, p_bar_hat_flat, input_P_flat = np.split(y, [4, 7])
        qua_hat_flat = qua_hat_flat/np.linalg.norm(qua_hat_flat)
        input_R_hat = self.rodrigues_formula(qua_hat_flat)
        ############ Quaternion ############
        ####################################

        ####################################
        ######### rotation matrix ##########
        # input_R_hat_flat, p_bar_hat_flat, input_P_flat = np.split(y, [9, 12])
        # input_R_hat = input_R_hat_flat.reshape((3,3))
        ######### rotation matrix ##########
        ####################################

        input_p_bar_hat = p_bar_hat_flat
        input_P = input_P_flat.reshape((6,6))

        # (self.k, z, self.q, self.Q, self.V, self.l)
        no_bar_vel = self.remove_bar(input_R_hat, self.linearVelocity)
        ####################################
        ####################################
        input_A = self.function_A()
        # print("input_A \n", input_A)
        # print("===========================")
        if len(self.z) != 0:
            input_C = self.function_C(input_R_hat)
        # print("input_C \n", input_C)
        # print("===========================")
        ####################################
        ####################################

        ####################################
        ############# Observer #############
        output_omega_hat_p_bar_hat_dot = self.observer_equations(input_p_bar_hat, input_R_hat, input_P)
        # print("output_omega_hat_p_bar_hat_dot \n", output_omega_hat_p_bar_hat_dot)
        # print("===========================")
        ############# Observer #############
        ####################################
        
        if len(self.z) != 0:
            output_P_dot = np.matmul(input_A, input_P) + np.matmul(input_P, np.transpose(input_A)) - np.matmul(input_P, np.matmul(np.transpose(input_C), np.matmul(self.Q, np.matmul(input_C, input_P)))) + self.V
        else:
            output_P_dot = np.matmul(input_A, input_P) + np.matmul(input_P, np.transpose(input_A)) + self.V
        p_bar_hat_dot = output_omega_hat_p_bar_hat_dot[3:]
        # print("output_P_dot \n", output_P_dot)
        # print("===========================")
        omega_hat = output_omega_hat_p_bar_hat_dot[0:3]

        ####################################
        ######### rotation matrix ##########
        # output_R = np.matmul(input_R_hat, self.function_S(omega_hat)).flatten()
        # return np.concatenate((output_R, p_bar_hat_dot, output_P_dot.flatten()))
        ######### rotation matrix ##########
        ####################################

        ####################################
        ############ Quaternion ############
        omega_hat_4x4 = np.array([[0, -omega_hat[0], -omega_hat[1], -omega_hat[2]],
                                [omega_hat[0], 0, omega_hat[2], -omega_hat[1]],
                                [omega_hat[1], -omega_hat[2], 0, omega_hat[0]],
                                [omega_hat[2], omega_hat[1], -omega_hat[0], 0]])
        output_qua_hat_flat = 0.5*np.matmul(omega_hat_4x4, qua_hat_flat)
        # print("np.concatenate((output_qua_hat_flat, p_bar_hat_dot, output_P_dot.flatten())) \n", np.concatenate((output_qua_hat_flat, p_bar_hat_dot, output_P_dot.flatten())))
        # print("===========================")
        return np.concatenate((output_qua_hat_flat, p_bar_hat_dot, output_P_dot.flatten()))
        ########### Quaternion ############
        ####################################

    def rk45_step(self):
        
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
        safety_factor = 0.8
        min_scale_factor = 0.2
        max_scale_factor = 10.0
        if error_norm <= self.tol:
            success = True
            t = self.current_time+self.dt
            dt = self.dt * min(max_scale_factor, max(min_scale_factor, safety_factor * (self.tol / error_norm)**0.25))
        else:
            success = False
            t = -1
            dt = self.dt * max(min_scale_factor, safety_factor * (self.tol / error_norm)**0.25)

        return y_next, t, dt, success

    def step_simulation(self):
        ######################################################
        ####################### Solver #######################
        self.running_rk45 = True
        success = False
        # self.dt = self.stepsize
        while not success:
            y_next, next_time, new_dt, success = self.rk45_step()
            if success:
                self.soly = y_next
                self.solt = next_time

                self.running_rk45 = False
                # print('current_time', self.current_time, self.dt, self.l)
            else:
                # print("---------------- Failed ----------------")
                pass
            self.dt = new_dt
        ####################### Solver #######################
        ######################################################
        return (self.solt, self.dt, self.soly, self.ErrMsg)