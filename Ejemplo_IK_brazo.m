%% IMPORTANTE, SE ASUME QUE PRIMERO SE HA EJECUTADO "Ejemplo_movimiento_brazo.m"

% Concectar con máquina virtual ROS
% ipaddress = '192.168.232.128';
% rosinit(ipaddress);

% Cargar el robot PR2 como un objeto tipo robotics.RigidBodyTree
pr2 = exampleHelperWGPR2Kinect;

% Crear un suscriptor a un topic para leer las posiciones de las
% articulaciones
jointSub = rossubscriber('joint_states');
jntState = receive(jointSub);

% Mostrar por pantalla el robotics.RigidBodyTree
jntPos = exampleHelperJointMsgToStruct(pr2,jntState);
   

% Descomentar para visualizar en Matlab
% show(pr2,jntPos);

% Como para el movimiento del brazo no se quiere tener en cuenta la
% movilidad del tronco, se limitan sus posiciones. 
torsoJoint = pr2.getBody('torso_lift_link').Joint;
idx = strcmp({jntPos.JointName}, torsoJoint.Name);
torsoJoint.HomePosition = jntPos(idx).JointPosition;
torsoJoint.PositionLimits = jntPos(idx).JointPosition + [-1e-3,1e-3];

% Se crea un objeto de cinemática inversa basado en el robotics.RigidBodyTree
ik = robotics.InverseKinematics('RigidBodyTree', pr2);

% La cinematica inversa funciona por optimizacion. 
% Para que empiece la optimizacion, puede usar una posicion articular
% aleatoria, o podemos darle un posicion inicial.
% Para este caso, desactivamos la posicion de inicio aleatoria. 
ik.SolverParameters.AllowRandomRestart = false;
% Se establece la posicion inicial de la optimizacion como la posicion
% actual. 
initialGuess = jntPos;

% Se especifican las tolerancias para el alcance de cada posicion objetivo.
weights = [0.25 0.25 0.25 1 1 1];

% Especificar las posiciones objetivo para el brazo. 
% El brazo tratará de alcanzar una posicion de pre-agarre.
% Despues una pose de agarre, cerrará el gripper y tratará de mover la lata
% a otra posicion. 
% Es necesario especificar el frame con respecto al que se especifian las
% poses a alcanzar
endEffectorName = 'r_gripper_tool_frame';

% Posiciones y orientaciones para alcanzar para el agarre
TCanInitial = trvec2tform([0.7, 0.0, 0.55]);
TCanFinal = trvec2tform([0.6, -0.5, 0.55]);
TGraspToCan = trvec2tform([0,0,0.08])*eul2tform([pi/8,0,-pi]);
TGrasp = TCanInitial*TGraspToCan; 
T1 = TGrasp*trvec2tform([0.,0,-0.1]);
T2 = TGrasp*trvec2tform([0,0,-0.2]);
T3 = TCanFinal*TGraspToCan*trvec2tform([0,0,-0.2]);
TRelease = TCanFinal*TGraspToCan; 
T4 = T3*trvec2tform([-0.1,0,0]);
motionTask = {'Release', T1, TGrasp, 'Grasp', T2, T3, TRelease, 'Release', T4};

%% La trayectoria de agarre se ejecuta como sigue:
% - Abrir la pinza
% - Mover la pinza desde la posicion actual a T1
% - Mover la pinza desde la posicion actual a TGrasp
% - Cerrar la pinza
% - Levantar el brazo hasta T2
% - Mover la pinza a T3 sin cambiar el nivel
% - Mover a la posicion de soltar, TRelease
% - Abrir la pinza
% - Mover el brazo a la posicion T4

%%
% El movimiento real se ejecuta como puntos de una trayectoria articular
% segun las posiciones calculadas por la cinematica inversa 
% interpolando entre las posicines objetivo

for i = 1: length(motionTask)
    
    if strcmp(motionTask{i}, 'Grasp')
        exampleHelperSendPR2GripperCommand('right',0.0,1000,true); 
        continue
    end
    
    if strcmp(motionTask{i}, 'Release')
        exampleHelperSendPR2GripperCommand('right',0.1,-1,true);
        continue
    end  
    
    Tf = motionTask{i};
    % Obtener la posicion actual
    jntState = receive(jointSub);
    jntPos = exampleHelperJointMsgToStruct(pr2, jntState);
    
    T0 = getTransform(pr2, jntPos, endEffectorName);  
    
    % Interpolar entre puntos objetivo
    numWaypoints = 10;
    TWaypoints = exampleHelperSE3Trajectory(T0, Tf, numWaypoints); 
    jntPosWaypoints = repmat(initialGuess, numWaypoints, 1); 
    
    rArmJointNames = rGoalMsg.Trajectory.JointNames;
    rArmJntPosWaypoints = zeros(numWaypoints, numel(rArmJointNames));
    
    % Calcular la posicion articular usando la cinematica inversa
    for k = 1:numWaypoints
        jntPos = ik(endEffectorName, TWaypoints(:,:,k), weights, initialGuess);
        jntPosWaypoints(k, :) = jntPos;
        initialGuess = jntPos;
        
        % Extraer la posicion articular 
        rArmJointPos = zeros(size(rArmJointNames));
        for n = 1:length(rArmJointNames)
            rn = rArmJointNames{n};
            idx = strcmp({jntPos.JointName}, rn);
            rArmJointPos(n) = jntPos(idx).JointPosition;
        end  
        rArmJntPosWaypoints(k,:) = rArmJointPos'; 
    end
    
    % Interpolacion temporal 
    timePoints = linspace(0,3,numWaypoints);       
    % Calcular la velocidad de la trayectoria
    h = diff(timePoints); h = h(1);
    jntTrajectoryPoints = repmat(rosmessage('trajectory_msgs/JointTrajectoryPoint'),1,numWaypoints);
    [~, rArmJntVelWaypoints] = gradient(rArmJntPosWaypoints, h);
    for m = 1:numWaypoints
        jntTrajectoryPoints(m).Positions = rArmJntPosWaypoints(m,:);
        jntTrajectoryPoints(m).Velocities = rArmJntVelWaypoints(m,:);
        jntTrajectoryPoints(m).TimeFromStart = rosduration(timePoints(m));
    end
    
    % Descomentar para visualizar en Matlab
%     hold on
%     for j = 1:numWaypoints
%         show(pr2, jntPosWaypoints(j,:),'PreservePlot', false);
%         exampleHelperShowEndEffectorPos(TWaypoints(:,:,j));
%         drawnow;
%         pause(0.1);
%     end
    
    % Enviar la trayectoria articular a ROS
    rGoalMsg.Trajectory.Points = jntTrajectoryPoints;
    action.sendGoalAndWait(rGoalMsg);

end

% Mover el brazo hacia atrás
exampleHelperSendPR2GripperCommand('r',0.0,-1)
rGoalMsg.Trajectory.Points = tjPoint2;
sendGoal(rArm, rGoalMsg);

% rosshutdown





