#!/usr/bin/env python3
from datetime import datetime
from fractions import Fraction
import rospy
from inverted_pendulum_sim.msg import ControlForce, CurrentState
from inverted_pendulum_sim.srv import SetParams
from math import sin, pi



class Force_Generator():

    def __init__(self):
        rospy.init_node('Sinusoidal_Wave_Input_Force', anonymous=False)

        self.control_force_pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=10)
        self.current_state_sub = rospy.Subscriber('/inverted_pendulum/current_state', CurrentState, self.current_state_callback)
        
        self.control_force = 0
        self.current_state = CurrentState()

        self.start_time = datetime.now().timestamp()

        print("Provide Amplitude and Freequency : ")
        self.amplitude = int(input('AMPLITUDE : '))
        self.freequency = int(input('FREEQUENCY : '))

        self.main()
    
    def current_state_callback(self, msg):
        self.current_state = msg

    def print_data(self):
        print('x : '+ str(self.current_state.curr_x))
        print('force : '+ str(self.control_force))
        print('*****************')

    def sin_wave_force(self, AMPLITUDE, FREEQUENCY):
        self.current_time = datetime.now().timestamp()
        t= self.current_time-self.start_time

        force = AMPLITUDE*sin((2*pi*FREEQUENCY)*t)
        
        return force

    def main(self):
        while not rospy.is_shutdown():
            
            # Publishing force
            self.control_force=self.sin_wave_force(AMPLITUDE=self.amplitude, FREEQUENCY=self.freequency)
            self.control_force_pub.publish(self.control_force)

            # Print Representation
            self.print_data()

def main():
    InvertedPendulum = Force_Generator()

if __name__ == '__main__':
    main()