#  The phantom robot ROS practice.

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
#### Toolbox
![Pincher](https://user-images.githubusercontent.com/43300509/168179621-2b1af9be-f4fa-4307-aa42-6fe717d1df3a.svg)

This part of the lab was developed in Matlab; The model and forward kinematics were made with the Peter corke toolbox; the model showed below represents the analysis of the robot with the measurements obtained of the physical robot. The last rotative joint ris related at the gripper.


The DHstd parameters obtained due to this model:
![PincherDHstd](https://user-images.githubusercontent.com/43300509/168276069-649ced71-17cc-418f-860a-dc6874164650.png)

The table and model were ploted by the Matlab code below. Also the tool was added, just in order to represent the gripper, the phantom has.
```Matlab
l1= 4.45 ;
l2 = 10.49;
l3 = 10.7;
l4 = 8.41;
q = [pi/3 -pi/4 pi/2 pi/4 pi/6 ];
L(1) = Link('revolute','alpha', pi/2 , 'a', 0,   'd', l1 , 'offset', 0 , 'qlim', [-pi pi]);
L(2) = Link('revolute', 'alpha' , 0 ,'a', l2 , 'd', 0 , 'offset', pi/2, 'qlim',[-pi pi]);
L(3) = Link( 'revolute','alpha', 0 ,    'a',l3 ,   'd',0 ,   'offset', 0, 'qlim', [-pi pi]);
L(4) = Link( 'revolute','alpha', -pi/2,    'a', 0  ,   'd',0 ,   'offset', 0, 'qlim', [-pi pi]);
L(5) = Link( 'revolute','alpha', 0 ,    'a',l4 ,   'd',0 ,   'offset', 0 , 'qlim', [-pi pi]);

TCP = trotz(-pi/2)*trotx(-pi/2);
Pincher = SerialLink(L,'name','Pincher','tool',TCP)
Pincher.plot( q ,'jaxes','noa');
  
```
Now the next step is stablish a connection with the dynamixel_workbench, ROS topics and services. 

```Matlab
  
motorSvcClient = rossvcclient('/dynamixel_workbench/dynamixel_command'); %Creación de cliente de pose y posición
motorCommandMsg = rosmessage(motorSvcClient); %Creación de mensaje
sub = rossubscriber('/dynamixel_workbench/joint_states');

```
In the last code showed, the connection was made also with a node master due to a new node that Matlab creates, moreover, with the service 'dynamixel_command', this one is able to send messages to the workbench, in order to move the  robot, control the torque and other features.

Now such as the python script, the robot can be move with the address name 'Goal_Position' and send the  msg with the call() funtion, also the msg from Jointstates can be read with the receive() function ,afterwards the model can be ploted; in this way, the robot can be shown in the screen in his current position. 
```Matlab

for i = 1:5 
    for j = 1:5

        motorCommandMsg.AddrName = "Goal_Position";
        motorCommandMsg.Id = j;
        motorCommandMsg.Value = qs(i,j)* 3.4 + 512;
        call(motorSvcClient,motorCommandMsg);
        pause(1)
    end
        [tf,status,~] = receive(sub,10);
    
        if status
            q = tf.Position;
            Pincher.plot( q(1:5)','jaxes','noa');
            axis([-56 58 -57 57 -57 57])
        end
        pause(2)
end
```
### Conclusion

This Lab has join plenty of topics about ROS, check how it works, during the development of this code was important to check the information on some important topics with the echo command. Moreover the code provided by the px_repository; the URDF files and the description of every single Link and joint. 

The launcher files also were quite important, those elements had been useful, but at the same time due to poor domain it was so difficult to correct the problems in there and implement some others applications. At the end, reading just the 'px_collisions' URDF file and running the px_rviz_dyna.launch' file the issues were resolved; in fact, with the acknowledgment of the functionalities, the scripts were able to control, move and  program the phantom robot and the digital twin at the same time.
## Acknowledgements

 - [Felipe Gonzales (Professor)](https://felipeg17.github.io/index.html)
 - [Manuel Lugo (Monitor)](https://github.com/mlugom)
 - [Sergio Alejandro Triana (contributor)](https://github.com/alejotriana1)
