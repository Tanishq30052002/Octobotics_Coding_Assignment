#!/usr/bin/env python3
import rospy
from inverted_pendulum_sim.msg import ControlForce, CurrentState
from inverted_pendulum_sim.srv import SetParams
from math import pi
from simple_pid import PID
from std_msgs.msg import Float64

class InvertedPendulumController():

    def __init__(self):
        rospy.init_node('InvertedPendulumControl', anonymous=False)

        self.control_force_pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=10)
        self.current_state_sub = rospy.Subscriber('/inverted_pendulum/current_state', CurrentState, self.current_state_callback)
        self.theta_error_pub = rospy.Publisher('/inverted_pendulum/theta_error', Float64, queue_size=10)
        
        self.current_state = CurrentState()

        self.main()

    def sys_parameters(self):
        # sys
        self.p_mass=2
        self.p_length=300
        self.c_mass=0.5

        # cart motion : x, v, a
        self.c_x=0
        self.c_v=0
        self.c_a=0

        # pendulum motion : theta, omega, alpha
        self.p_theta = 3.1
        self.p_omega = 0
        self.p_alpha = 0

    def setParams(self):
        rospy.wait_for_service('/inverted_pendulum/set_params')
        try:
            set_params = rospy.ServiceProxy('/inverted_pendulum/set_params', SetParams)
            providing_params = set_params(  pendulum_mass = self.p_mass, pendulum_length = self.p_length, cart_mass = self.c_mass,
                                            cart_x_0 = self.c_x, cart_x_dot_0 = self.c_v, cart_x_dot_dot_0 = self.c_a,
                                            theta_0 = self.p_theta, theta_dot_0 = self.p_omega, theta_dot_dot_0 = self.p_alpha)
            return providing_params.success, providing_params.message
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
    
    def current_state_callback(self, msg):
        self.current_state = msg

    def pid_controller(self, theta):
        
        kp=8
        ki=200000
        kd=20000

        sample_time=0.0001
        desired_value=pi
        current_value=theta

        pid=PID(kp, ki, kd, sample_time, desired_value)
        force_magnitude=pid(current_value)

        if(theta<pi):
            force=-force_magnitude
        else:
            force=force_magnitude
        
        print('theta : '+str(current_value))
        print('force : '+str(force))
        print('error : '+str(current_value-desired_value))
        print('*****************')

        self.theta_error_pub.publish(current_value-desired_value)
        return force        

    def main(self):
        self.sys_parameters()
        self.setParams()
        while not rospy.is_shutdown():
            
            # Conversion of theta
            if(self.current_state.curr_theta<0):
                theta = self.current_state.curr_theta+2*pi
            else:
                theta = self.current_state.curr_theta

            # Publishing force
            self.control_force=self.pid_controller(theta)
            self.control_force_pub.publish(self.control_force)

            rospy.Rate(100000).sleep()
def main():
    InvertedPendulum = InvertedPendulumController()

if __name__ == '__main__':
    main()