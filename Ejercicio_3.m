% Concectar con máquina virtual ROS
ipaddress = '192.168.255.128';
rosinit(ipaddress);

% Crear el clinte de accion
% Además del cliente, se recibe como salida el tipo de mensaje a utilizar
% action = /head_traj_controller/point_head_action


% Esperar a que el servidor este activo


% Rellenar el mensaje con los datos correspondientes


% Enviar la accion al servidor y esperar respuesta


% Comprobar respuesta
if ActionClient.GoalState() == 'succeeded'
    disp('Succeeded')
else
    disp('Failed')
end

% rosshutdown 
