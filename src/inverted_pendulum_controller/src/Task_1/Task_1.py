#!/usr/bin/env python3

# Parameters to set
""" 
# car and pendulum mass

pendulum_mass = 2kg
pendulum_length = 300unit
cart_mass = 0.5kg

# cart position in the center
# x, v, a

cart_x_0 = 
cart_x_dot_0 = 0
cart_x_dot_dot_0 = 0

# pendulum orientation is vertical down
# theta, omega, alpha

theta_0 = 0
theta_dot_0 = 0
theta_dot_dot_0 = 0

"""

import rospy
from inverted_pendulum_sim.srv import SetParams


class InvertedPendulumController():
    def __init__(self):
        rospy.init_node('Set_Parameters', anonymous=False)

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
        self.p_theta = 0
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
    
def main():
    InvertedPendulum = InvertedPendulumController()
    InvertedPendulum.sys_parameters()
    InvertedPendulum.setParams()
    rospy.spin()

if __name__ == '__main__':
    main()