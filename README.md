#  The phantom robot ros practice.

This repository is going to show the results and procedures of the Ros mechanics, in this way, The repository shows the commands and 
scripts to make a correct use of the phantom x robots.


## ROS 

#### 
In order to send commands and stablish comunication with the robot, it is need to import the following libraries, nodes and msg from Dynamixel and ROS.  the Dynamixel package is the workbench
that controls the phantom x btw.


```python
import rospy
import time
from pynput.keyboard import Key, Listener, KeyCode
from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand
import numpy as np
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
  
```
It is important to check also the .Launch and .Yaml file in the px_robot repository.
So it is important to mention, as advice, that the motors (FT232) have to be recognized in the lsusb terminal command  and they have to receive permissions to be used
Then, the motors can be used in the dynamixel workbench and ROS with the px_controllers launcher found in px_robot repository.

After all of the previous settings, the LAB2.py is launched with the  px_rviz_dyna.launch file.
In order to move every joint in the phantom robot, the W key is used to change from 1st joint  to the 4th Joint.
The code used for this key the action was defined in the class 'Pincher' and method 'on_press'.
```python
def on_press(self, key):

        if key == KeyCode.from_char('w'):
            if self.a < 4:
                self.a += 1
            else:
                self.a = 1
            print('articulacion:' )
            print(self.a)
            

```
And it's used the same algorithm for the key S; for the A and d Keys its set the home position and movement position 
with the home and goal values:
```python
        self.home = [512, 512, 512, 512]
        self.goal = [720, 720, 250, 720]
if key == KeyCode.from_char('d'):

            jointCommand('',self.a,'Goal_Position',self.home[self.a-1],0.2)
            # self.pub(self.a, self.home[self.a-1])

        if key == KeyCode.from_char('a'):

            jointCommand('',self.a,'Goal_Position',self.goal[self.a-1],0.2)
            # self.pub(self.a, self.goal[self.a-1])

```
Additionally the comments in the methods were made to publish, receive messages from the joint/states topic.
For that reason, the currrent configuration is posted in Rviz without connection/access to the phantom x.
The connection with the joint states is showed in the following method:
```python
def pub(self, id_num, value):
        
        printer = rospy.Publisher('/joint_states', JointState, queue_size=10)

        q=[0,0,0,0]
        q[id_num-1]=np.deg2rad(round(value/1023 * 300 - 150))

        msg = JointState()
        msg.header.stamp = rospy.Time().now()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        msg.position = q
        printer.publish(msg)


```
Finally the code was ran with torque parameters:
```python
if __name__ == '__main__':
    # rospy.init_node('joint_node', anonymous=False)
    p = Pincher()
    try:
        # Goal_Position (0,1023)
        # Torque_Limit (0,1023)
        jointCommand('', 1, 'Torque_Limit', 600, 0)
        jointCommand('', 2, 'Torque_Limit', 550, 0)
        jointCommand('', 3, 'Torque_Limit', 500, 0)
        jointCommand('', 4, 'Torque_Limit', 500, 0)
        with Listener(on_press=p.on_press) as listener:
            listener.join()

    except rospy.ROSInterruptException:
        pass
```
