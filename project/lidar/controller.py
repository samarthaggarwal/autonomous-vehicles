import rospy
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from scipy.integrate import ode
from std_msgs.msg import Float32MultiArray

def func1(t, vars, vr, delta):
    curr_x = vars[0]
    curr_y = vars[1]
    curr_theta = vars[2]

    dx = vr * np.cos(curr_theta)
    dy = vr * np.sin(curr_theta)
    dtheta = delta
    return [dx,dy,dtheta]


class bicycleModel():

    def __init__(self, velocity = 10, deceleration = 0):
        self.waypointSub = rospy.Subscriber("/gem/waypoint", ModelState, self.__waypointHandler, queue_size=1)
        self.waypointPub = rospy.Publisher("/gem/waypoint", ModelState, queue_size=1)
        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.controlPub = rospy.Publisher("/gem/control", Float32MultiArray, queue_size = 1)

        init_state = self.getModelState()

        self.v_0 = velocity
        self.v_1 = self.v_0
        self.deceleration = deceleration
        self.stopped = False
        self.x = init_state.pose.position.x
        self.y = 0

    def getModelState(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            modelState = serviceResponse(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
        return modelState

    def rearWheelModel(self, ackermannCmd):
        currentModelState = self.getModelState()

        if not currentModelState.success:
            return

        vr = ackermannCmd.speed
        delta = ackermannCmd.steering_angle_velocity

        x = currentModelState.pose.position.x
        y = currentModelState.pose.position.y
        euler = self.quaternion_to_euler(currentModelState.pose.orientation.x,
                                    currentModelState.pose.orientation.y,
                                    currentModelState.pose.orientation.z,
                                    currentModelState.pose.orientation.w)
        theta = euler[2]

        initR = [self.x,self.y,theta]
        r = ode(func1)
        r.set_initial_value(initR)
        r.set_f_params(vr,delta)
        val = r.integrate(r.t+0.01)

        new_x = val[0]
        new_y = val[1]
        new_theta = val[2]
        self.x = new_x
        self.y = new_y
        return [new_x, new_y, new_theta]

    def rearWheelFeedback(self, currentPose, targetPose, vehicle_state = "run"):
        #Gain Values
        k1 = 1
        k2 = 1
        k3 = 1
        curr_x = currentPose.pose.position.x
        curr_y = currentPose.pose.position.y

        currentEuler = self.quaternion_to_euler(currentPose.pose.orientation.x,
                                           currentPose.pose.orientation.y,
                                           currentPose.pose.orientation.z,
                                           currentPose.pose.orientation.w)

        curr_x = currentPose.pose.position.x
        curr_y = currentPose.pose.position.y
        curr_theta = currentEuler[2]

        targ_x = targetPose.pose.position.x
        targ_y = targetPose.pose.position.y

        error_x = targ_x - curr_x
        error_y = targ_y - curr_y
        error_theta = (curr_theta-(np.arctan2(error_y,error_x)%(2*np.pi)))%(np.pi*2)
        if error_theta > np.pi:
            error_theta = error_theta - np.pi*2
        # vr = 10*np.sqrt(error_x**2+error_y**2)

        if vehicle_state == "run":
            vr = self.v_1
            # self.v_1 = self.v_0
        elif vehicle_state == "brake":
            vr = self.v_1
            self.v_1 = max(self.v_1-self.deceleration*0.01,0)
        delta = -4*error_theta

        if delta>np.pi/2:
            delta = np.pi/2
        elif delta<-np.pi/2:
            delta = -np.pi/2


        #give blank ackermannCmd
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = vr
        newAckermannCmd.steering_angle_velocity = delta

        return newAckermannCmd

    def setModelState(self, currState, targetState, vehicle_state = "run"):
        control = self.rearWheelFeedback(currState, targetState, vehicle_state)
        a = Float32MultiArray()
        a.data = [control.speed,control.steering_angle_velocity]
        self.controlPub.publish(a)
        values = self.rearWheelModel(control)

        # print(values[1], control.speed)
        if control.speed <= 0 and not self.stopped:
            self.stopped = True
            text = 'vehicle stopped x ' + str(values[0])+' y '+str(values[1])
            print(text)

        newState = ModelState()
        newState.model_name = 'gem'
        newState.pose = currState.pose
        newState.pose.position.x = values[0]
        newState.pose.position.y = values[1]
        newState.pose.position.z = 0.01
        q = self.euler_to_quaternion([values[2],0,0])
        newState.pose.orientation.x = q[0]
        newState.pose.orientation.y = q[1]
        newState.pose.orientation.z = q[2]
        newState.pose.orientation.w = q[3]
        newState.twist.linear.x = 0
        newState.twist.linear.y = 0
        newState.twist.linear.z = 0
        newState.twist.angular.x = 0
        newState.twist.angular.y = 0
        newState.twist.angular.z = 0
        self.modelStatePub.publish(newState)

    def stopMoving(self):
        control = AckermannDrive()
        control.speed = 0
        control.steering_angle_velocity = 0

        a = Float32MultiArray()
        a.data = [control.speed, control.steering_angle_velocity]

        self.controlPub.publish(a)

        currState = self.getModelState()
        newState = ModelState()

        newState.model_name = 'gem'
        newState.pose = currState.pose
        newState.twist.linear.x = 0
        newState.twist.linear.y = 0
        newState.twist.linear.z = 0
        newState.twist.angular.x = 0
        newState.twist.angular.y = 0
        newState.twist.angular.z = 0
        self.modelStatePub.publish(newState)

    def euler_to_quaternion(self, r):
        (yaw, pitch, roll) = (r[0], r[1], r[2])
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        return [roll, pitch, yaw]

    def __waypointHandler(self, data):
        self.waypointList.append(data)
