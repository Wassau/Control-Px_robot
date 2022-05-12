import rospy
import time
from pynput.keyboard import Key, Listener, KeyCode
from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand
import numpy as np
from std_srvs.srv import Empty


class Pincher:
    self.a = 1;
    self.home = [512, 512,512,512,512]
    self.goal = [720, 720,250,720,250]
    def on_press(self,key):

        if key == KeyCode.from_char('w'):
            self.a = 1
            if self.a < 5 :
                self.a+=1
            else:
                self.a = 0
        print(a)
        if key == KeyCode.from_char('s'):
            if self.a > 1 :
                self.a-=1
            else:
                self.a = 5
        print(a)
            
        if key == KeyCode.from_char('d'):
            
            jointCommand('',self.a),'Goalposition',self.home[self.a],0.2)

        if key == KeyCode.from_char('a'):

            jointCommand('',self.a),'Goalposition',self.goal[self.a],0.2)            

def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

if __name__ == '__main__':
    try:
        # Goal_Position (0,1023)
        # Torque_Limit (0,1023)
        jointCommand('', 1, 'Torque_Limit', 600, 0)
        jointCommand('', 2, 'Torque_Limit', 400, 0)
        jointCommand('', 3, 'Torque_Limit', 400, 0)
        jointCommand('', 4, 'Torque_Limit', 400, 0)
        
          with Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()
        jointCommand('',a,)
        
    except rospy.ROSInterruptException:
        pass