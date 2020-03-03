% Concectar con máquina virtual ROS
% ipaddress = '192.168.232.128';
% rosinit(ipaddress);

% Crear el clinte de accion
% Además del cliente, se recibe como salida el tipo de mensaje a utilizar
% action = r_arm_controller/joint_trajectory_action
[action, rGoalMsg] = rosactionclient('r_arm_controller/joint_trajectory_action');
waitForServer(action);

% Rellenar con los nombres de las articulaciones que se deben mover
rGoalMsg.Trajectory.JointNames = {'r_shoulder_pan_joint', ...
                                  'r_shoulder_lift_joint', ...
                                  'r_upper_arm_roll_joint', ...
                                  'r_elbow_flex_joint',...
                                  'r_forearm_roll_joint',...
                                  'r_wrist_flex_joint',...
                                  'r_wrist_roll_joint'};

% Point 1
tjPoint1 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint1.Positions = zeros(1,7);
tjPoint1.Velocities = zeros(1,7);
tjPoint1.TimeFromStart = rosduration(2.0);

% Point 2
tjPoint2 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint2.Positions = [-1.0 0.2 0.1 -1.2 -1.5 -0.3 -0.5];
%tjPoint2.Positions = [-0.5 0.2 0.1 -0.5 -0.5 -0.3 -0.5];
tjPoint2.Velocities = zeros(1,7);
tjPoint2.TimeFromStart = rosduration(3.0);

% Rellenar el mensaje con los puntos de la trayectoria
rGoalMsg.Trajectory.Points = [tjPoint1,tjPoint2];

% Enviar la accion al servidor y esperar respuesta
action.sendGoalAndWait(rGoalMsg);                               
  
% Escribir el resultado
disp(action.GoalState())
    

% rosshutdown





