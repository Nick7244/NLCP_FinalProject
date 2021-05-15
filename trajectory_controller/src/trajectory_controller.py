#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
from scipy.linalg import expm
import ur5
from ur5_box_msgs.msg import ur5_trajectory
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class TrajectoryControllerNode:
    def __init__(self):
        # Finding current directory
        rospack = rospkg.RosPack()
        rospack.list()
        dir = rospack.get_path('trajectory_controller')

        self.executingTraj = False

        # Create UR5 Calculator Object
        self.ur5Calculator = ur5.robot_config(dir + '/data')

        # Initialize the Mq and Mq_g matricies
        self.ur5Calculator.Mq([0,0,0,0,0,0])
        self.ur5Calculator.Mq_g([0,0,0,0,0,0])

        self.Ko = np.diag([5, 5, 5, 5, 5, 5])
        self.K = np.diag([5, 5, 5, 5, 5, 5])

        self.joint_lookup = {'shoulder_pan_joint':'1', 'shoulder_lift_joint':'2', 'elbow_joint':'3', 
                             'wrist_1_joint':'4', 'wrist_2_joint':'5', 'wrist_3_joint':'6'}

        self.trajectory = {'pos':{'1':[], '2':[], '3':[], '4':[], '5':[], '6':[]}, 
                           'vel':{'1':[], '2':[], '3':[], '4':[], '5':[], '6':[]},
                           'acc':{'1':[], '2':[], '3':[], '4':[], '5':[], '6':[]}}
        self.trajectory_len = 0
        self.joint_state = {'pos':{'1':0.1, '2':0.0, '3':0.0, '4':0.0, '5':0.0, '6':0.0},
                            'vel':{'1':0.0, '2':0.0, '3':0.0, '4':0.0, '5':0.0, '6':0.0}}

        # State to maintain between executing trajectories
        self.stable_state = {'pos':{'1':0.0, '2':0.0, '3':0.0, '4':0.0, '5':0.0, '6':0.0}}
        for j in ['1', '2', '3', '4', '5', '6']:
            self.stable_state['pos'][j] = 0.0

        # Subscribe to joint positions / velocities feedbacks
        rospy.Subscriber("/ur5_seed_trajectory", ur5_trajectory, self.trajectoryCallback)
        rospy.Subscriber("/joint_states", JointState, self.jsUpdateCallback)

        # Publish to Joint Torques
        self.pub = rospy.Publisher("/control_policy_torques", JointState, queue_size=10)
    
    def trajectoryCallback(self, msg):
        # Save the trajectory
        self.trajectory['pos']['1'] = msg.q1_trajectory.data
        self.trajectory['pos']['2'] = msg.q2_trajectory.data
        self.trajectory['pos']['3'] = msg.q3_trajectory.data
        self.trajectory['pos']['4'] = msg.q4_trajectory.data
        self.trajectory['pos']['5'] = msg.q5_trajectory.data
        self.trajectory['pos']['6'] = msg.q6_trajectory.data

        self.trajectory['vel']['1'] = msg.q1_dot_trajectory.data
        self.trajectory['vel']['2'] = msg.q2_dot_trajectory.data
        self.trajectory['vel']['3'] = msg.q3_dot_trajectory.data
        self.trajectory['vel']['4'] = msg.q4_dot_trajectory.data
        self.trajectory['vel']['5'] = msg.q5_dot_trajectory.data
        self.trajectory['vel']['6'] = msg.q6_dot_trajectory.data

        self.trajectory['acc']['1'] = msg.q1_dot_dot_trajectory.data
        self.trajectory['acc']['2'] = msg.q2_dot_dot_trajectory.data
        self.trajectory['acc']['3'] = msg.q3_dot_dot_trajectory.data
        self.trajectory['acc']['4'] = msg.q4_dot_dot_trajectory.data
        self.trajectory['acc']['5'] = msg.q5_dot_dot_trajectory.data
        self.trajectory['acc']['6'] = msg.q6_dot_dot_trajectory.data

        # Update the trajectory length
        self.trajectory_len = len(self.trajectory['pos']['1'])

        # Execute the Trajectory
        self.executeTrajectory()

    def jsUpdateCallback(self, msg):
        for i in range(6):
            joint_num_str = self.joint_lookup[msg.name[i]]
            self.joint_state['pos'][joint_num_str] = msg.position[i]
            self.joint_state['vel'][joint_num_str] = msg.velocity[i]
        
        return
    
    def feedbackControl(self, y, dy, yd, dyd, ddyd):

        #print(y-yd)
        
        mod_y = y.copy()
        #mod_y[1] = mod_y[1] - np.pi / 2
        #mod_y[4] = mod_y[4] - np.pi / 2

        M = self.ur5Calculator.Mq(mod_y.flatten())
        g = self.ur5Calculator.Mq_g(mod_y.flatten())

        g = np.expand_dims(g, axis=1)
        u = np.matmul(M, (ddyd - (y-yd) - np.matmul(self.Ko, (dy - dyd)) - np.matmul( self.K, dy - dyd + np.matmul(self.Ko, (y-yd)) ) )) + g

        return u

    
    def executeTrajectory(self):
        self.executingTraj = True

        rate = rospy.Rate(100) # 100 Hz loop
        for i in range(self.trajectory_len):

            # Create column vectors containing the data
            y = np.array([[self.joint_state['pos'][joint] for joint in ['1', '2', '3', '4', '5', '6']]]).T
            dy = np.array([[self.joint_state['vel'][joint] for joint in ['1', '2', '3', '4', '5', '6']]]).T
            yd = np.array([[self.trajectory['pos'][joint][i] for joint in ['1', '2', '3', '4', '5', '6']]]).T
            dyd = np.array([[self.trajectory['vel'][joint][i] for joint in ['1', '2', '3', '4', '5', '6']]]).T
            ddyd = np.array([[self.trajectory['acc'][joint][i] for joint in ['1', '2', '3', '4', '5', '6']]]).T

            # Get the control values (Nm)
            u = self.feedbackControl(y, dy, yd, dyd, ddyd)
            u = u.flatten()
            
            # Publish control values (torque, Nm)
            msg = JointState()
            msg.effort = list(u)
            self.pub.publish(msg)

            # Update the stable state
            for k in ['1', '2', '3', '4', '5', '6']:
                self.stable_state['pos'][k] = self.joint_state['pos'][k]

            rate.sleep()
        
        self.executingTraj = False
        return

    def maintain_position(self):

        # If we are executing a Trajectory, do not maintain stability
        if self.executingTraj:
            return

        # Create column vectors containing the data
        y = np.array([[self.joint_state['pos'][joint] for joint in ['1', '2', '3', '4', '5', '6']]]).T
        dy = np.array([[self.joint_state['vel'][joint] for joint in ['1', '2', '3', '4', '5', '6']]]).T
        yd = np.array([[self.stable_state['pos'][joint] for joint in ['1', '2', '3', '4', '5', '6']]]).T
        dyd = np.array([[0, 0, 0, 0, 0, 0]]).T
        ddyd = np.array([[0, 0, 0, 0, 0, 0]]).T

        # Get the control values (Nm)
        u = self.feedbackControl(y, dy, yd, dyd, ddyd)
        u = u.flatten()
        
        # Publish control values (torque, Nm)
        msg = JointState()
        msg.effort = list(u)
        self.pub.publish(msg)

        return


    """
    def generateInertiaMatrix(self, y):
        masses = []
        MOIs = []
        Jacobians = generateManipulatorJacobians(y)
        inertia = np.zeros((6,6))

        for i in range(6):
            inertia = intertia + mass


    def generateManipulatorJacobians(self, y, com_wrt_prev_joint):
        ws = [[0,0,1]]
        ws.append([0,1,0])
        ws.append([0,1,0])
        ws.append([0,1,0])
        ws.append([0,0,-1])
        ws.append([0,1,0])
        ws = np.array(ws).T

        qs = [[0,0,0]]
        qs.append([0,           0,                  89.159])
        qs.append([425,         0,                  89.159])
        qs.append([425+392.25,  0,                  89.159])
        qs.append([425+392.25,  135.85-119.7+93,    0])
        qs.append([425+392.25,  0,                  89.159-94.65])
        qs = np.array(qs).T

        xi = np.zeros((6,6))
        for i in range(ws.shape[1]):
            xi[i, 0:3] = - np.cross(w[i,:], q[i,:])
            xi[i, 3:6] = w[i,:]

        # Calculating Jacobian
        Jacobian = calculateXiPrime(self, 1, xi)
        Jacobian = np.block([Jacobian, calculateXiPrime(self, 2, xi, y)])
        Jacobian = np.block([Jacobian, calculateXiPrime(self, 3, xi, y)])
        Jacobian = np.block([Jacobian, calculateXiPrime(self, 4, xi, y)])
        Jacobian = np.block([Jacobian, calculateXiPrime(self, 5, xi, y)])
        Jacobian = np.block([Jacobian, calculateXiPrime(self, 6, xi, y)])

        Jacobian_to_Joints = []
        for i in range(6):


        return Jacobian
    
    def calculateXiPrime(self, n, xi, y):
        if n == 1:
            return xi[:,0]
        else:
            product = exptwist(xi[:,0], y[0])
            for i in range(1, n-1):
                product = product @ exptwist(xi[:,i], y[i])
            return adjoint(product) @ xi(n-1)

    def adjoint(self, matrix):
        R = matrix[0:3, 0:3]
        P = vecSkew(matrix[0:3, 3])
        zero = np.zeros((3,3))
        return np.block([[R,   P @ R],
                         [zero,    R]])

    def skew(self, vec):
        vecSkew = np.array([[      0,  -vec[2],   vec[1]],
                            [ vec[2],        0,  -vec[0]],
                            [-vec[1],   vec[0],        0]])
        return vecSkew
    
    def skewtwist(self, twist):
        return np.block([[skew(twist[3:6]), twist[0:3]], [0, 0, 0, 0]])


    def exptwist(self, twist, angle):
        rot = scipy.linalg.expm(skewtwist(twist) * angle)
    """

