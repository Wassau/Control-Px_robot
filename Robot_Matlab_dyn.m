
 
%%
rosinit; %Conexion con nodo maestro
%%
motorSvcClient = rossvcclient('/dynamixel_workbench/dynamixel_command'); %Creación de cliente de pose y posición
motorCommandMsg = rosmessage(motorSvcClient); %Creación de mensaje
%%


motorCommandMsg.AddrName = "Goal_Position";
motorCommandMsg.Id = 2;
motorCommandMsg.Value = 600;
call(motorSvcClient,motorCommandMsg);

sub = rossubscriber(topicname);

rosshutdown;