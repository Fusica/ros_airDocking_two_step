import numpy as np
from numpy.linalg import norm
import math
import pdb

GRAV = 9.81 

def clamp_norm(x, maxnorm):
    n = np.linalg.norm(x)
    return x if n <= maxnorm else (maxnorm / n) * x

def normalize(x):
    n = np.linalg.norm(x)
    if n < 0.00001:
        return x, 0
    return x / n, n

def cross(a, b):
    return np.array([a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]])

#旋转矩阵转欧拉角
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


def rotationMatrixToEulerAngles(R) :
    assert(isRotationMatrix(R))
    
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    
    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])


class NonlinearPositionController(object):
    def __init__(self, force_control=False):
        # jacobian = quadrotor_jacobian(dynamics)
        # self.Jinv = np.linalg.inv(jacobian)
        ## Jacobian inverse for our quadrotor
        # Jinv = np.array([[0.0509684, 0.0043685, -0.0043685, 0.02038736],
        #                 [0.0509684, -0.0043685, -0.0043685, -0.02038736],
        #                 [0.0509684, -0.0043685,  0.0043685,  0.02038736],
        #                 [0.0509684,  0.0043685,  0.0043685, -0.02038736]])
        self.action = None

        #self.kp_p, self.kd_p = 4.5, 3.5
        #self.kp_p, self.kd_p = 4.5, 3.5
        self.kp_p = np.array([4.5, 3.5, 6.4])
        self.kd_p = np.array([4.5, 3.5, 2])
        self.ki_p = np.zeros(3)

        self.kp_a, self.kd_a = 200.0, 70.0 #50.

        #contact_froce 
        self.force_control = force_control
        self.ki_force = 0.1#0.8
        self.e_force = np.zeros(3)

        self.rot_des = np.eye(3)
        # self.rot_des = np.array([[-1.,0.,0.],
        #                         [0.,-1.,0.],
        #                         [0.,0.,1]])

        #self.step_func = self.step

        # pidThrustOmega
        self.angle = np.zeros(3)
        self.last_angle = np.zeros(3)
        self.item_omega = np.zeros(3)
        circle_per_sec = 2* np.pi
        self.angle_p_x = 5.0
        self.angle_p_y = 5.0
        self.angle_p_z = 0.0075#0.008#0.015
        self.kpa = np.array([0.000035,8,0.0075]) #  np.array([0.000035,8,0.0075])
        #self.kda = np.array([0.,8,0.]) # np.array([0.,8,0.])
        self.angle_i = np.zeros(3)
        self.e_pi = np.zeros(3)
        # self.kpa = np.array([20,20,20])
        # self.kda = np.array([7.,7,7])

        max_rp =  0.1 * circle_per_sec
        max_yaw =  0.1 * circle_per_sec
        self.min_omega = np.array([ -max_rp, -max_rp, -max_yaw])
        self.max_omega = np.array([  max_rp,  max_rp,  max_yaw])

    def stepThrustOmega(self, dynamics, goal, action=None, dt=0.01, flag="body"):
        #print(f"goal is {goal}, pos is {dynamics.pos}")
        #print("stepthrustOmega")
        # self.kp_p = np.array([4.5, 3.5, 6.4])
        # self.kd_p = np.array([4.5, 3.5, 2])

        # self.kpa = np.array([2.5,  2.5, 6.])
        # self.kda = np.array([1.7, 0.7, 0.])
        
        self.kp_p = np.array([2.7, 4.0, 18.])
        self.kd_p = np.array([1.5, 3., 9.5]) 
        self.ki_p = np.array([0.005, 0.005, 0.06])
        
        # self.kpa = np.array([2.5,  2.5, 6.])
        # self.kda = np.array([1.7, 0.7, 0.])
        self.kpa = np.array([3.,  2.5, 6.])
        self.kda = np.array([0.8, 0.7, 0.])

        #self.ki_force = 0.1
        self.ki_force = 0.1

        to_goal = goal - dynamics.pos
        goal_dist = norm(to_goal)
        e_p = -clamp_norm(to_goal, 0.5)
        e_v = dynamics.vel
        
        #desired force 应该是在机体坐标系下进行描述的 所以需要转换成世界坐标系（需要缕一缕）
        #[Hybrid Force/Motion Control and Internal Dynamics of Quadrotors for Tool Operation] 
        #####################################################################################
        self.e_force += dynamics.contact_force - dynamics.desired_force_world
        force_item = dynamics.desired_force_world - self.ki_force * self.e_force * dt #dynamics.contact_torque
        force_item[0] = 0.
        force_item[2] = 0.
        if not self.force_control or (not np.any(dynamics.contact_force)): #判断是否接触力为0
            force_item[1] = 0.
        #force_item[1] = 0.
        acc_des = -self.kp_p * e_p - self.kd_p * e_v + np.array([0, 0, GRAV]) - force_item/dynamics.mass
        
        
        #acc_des = -self.kp_p * e_p - self.kd_p * e_v + np.array([0, 0, GRAV])

        xc_des = self.rot_des[:, 0] 

        zb_des, _ = normalize(acc_des)
        yb_des, _ = normalize(cross(zb_des, xc_des))
        xb_des    = cross(yb_des, zb_des)
        R_des = np.column_stack((xb_des, yb_des, zb_des))
        R = dynamics.rot
        self.angle = rotationMatrixToEulerAngles(R)

        # des_omega = (self.angle - self.last_angle)/dt
        #print(R)
        def vee(R):
            return np.array([R[2,1], R[0,2], R[1,0]])
        e_R = vee(np.matmul(R_des.T, R) - np.matmul(R.T, R_des))
        e_R[2] *= 0.2 # slow down yaw dynamics
        e_w = dynamics.omega
        

        #dw_des = -self.kp_a * e_R - self.kd_a * e_w
        
        thrust_mag = np.dot(acc_des, R[:,2])

        item_thrust = thrust_mag / GRAV - 1

        dw_des_2 = -self.kpa * e_R - self.kda * e_w
        action = [item_thrust, dw_des_2[0], dw_des_2[1], dw_des_2[2]]
        new_action = np.array(action)
        return new_action


    
    def stepThrustOmega50hz(self, pos,vel, rot, omega, goal, mass=1.0):
        self.kp_p = np.array([0.75, 3.8, 18.])
        self.kd_p = np.array([0.95, 3., 9.5])
        self.ki_p = np.array([0.001, 0.001, 0.06])
        
        # self.kpa = np.array([2.5,  2.5, 6.])
        # self.kda = np.array([1.7, 0.7, 0.])
        self.kpa = np.array([3.8,  2.5, 6.])
        self.kda = np.array([0.8, 0.7, 0.])

        self.ki_force = 0.5

        to_goal = goal - pos
        goal_dist = norm(to_goal)
        e_p = -clamp_norm(to_goal, 4.0)
        e_v = vel
        self.e_pi += e_p
        
        #desired force 应该是在机体坐标系下进行描述的 所以需要转换成世界坐标系（需要缕一缕）
        #[Hybrid Force/Motion Control and Internal Dynamics of Quadrotors for Tool Operation] 
        #####################################################################################
        force_item = np.zeros(3)
        # self.e_force += contact_force - desired_force_world
        # force_item = desired_force_world - self.ki_force * self.e_force * dt #dynamics.contact_torque
        # force_item[0] = 0.
        # force_item[2] = 0.
        # if not self.force_control or (not np.any(contact_force)): #判断是否接触力为0
        #     force_item[1] = 0.
            
        #acc_des = -self.kp_p * e_p - self.kd_p * e_v + np.array([0, 0, GRAV]) - force_item/mass
        acc_des = -self.ki_p * self.e_pi - self.kp_p * e_p - self.kd_p * e_v + np.array([0, 0, GRAV]) - force_item/mass

        
        xc_des = self.rot_des[:, 0] 

        zb_des, _ = normalize(acc_des)
        yb_des, _ = normalize(cross(zb_des, xc_des))
        xb_des    = cross(yb_des, zb_des)
        R_des = np.column_stack((xb_des, yb_des, zb_des))
        R = rot
        #self.angle = rotationMatrixToEulerAngles(R)

        # des_omega = (self.angle - self.last_angle)/dt
        #print(R)
        def vee(R):
            return np.array([R[2,1], R[0,2], R[1,0]])
        e_R = vee(np.matmul(R_des.T, R) - np.matmul(R.T, R_des))
        e_R[2] *= 0.2 # slow down yaw dynamics
        e_w = omega
        

        #dw_des = -self.kp_a * e_R - self.kd_a * e_w
        
        thrust_mag = np.dot(acc_des, R[:,2])

        item_thrust = thrust_mag / GRAV - 1

        dw_des_2 = -self.kpa * e_R - self.kda * e_w
        action = [item_thrust, dw_des_2[0], dw_des_2[1], dw_des_2[2]]
        new_action = np.array(action)
        return new_action, _

pid_control = NonlinearPositionController()

def modelPredict(pos,vel, rot, omega, goal, mass=2.1):
    # import pdb
    #pdb.set_trace()
    action, _ = pid_control.stepThrustOmega50hz(pos, vel, rot, omega, goal,mass)
    return action
