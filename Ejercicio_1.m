% Concectar con máquina virtual ROS
ipaddress = '192.168.255.128';
rosinit(ipaddress);

% Crear publicador con los siguiente parametros:
% topic = /base_controller/command
% tipo de mensaje = geometry_msgs/Twist


% Crear un mensaje del tipo marcado por el publicador


% Hasta que se pare con "Ctrl+c"
while(1)
    % Pedir nueva tecla
    prompt = 'Elija movimiento (w,s,a,d,e,q): ';
    s = input(prompt,'s');
    % Dependiendo de la tecla, parametrizar movimiento
    if s == 'w'

    elseif s == 's'
         
    elseif s == 'd'

    elseif s == 'a'

    elseif s == 'q'

    elseif s == 'e'

    else  
        disp('Tecla incorrecta')
    end
    % Publicar mensaje
      
end

% rosshutdown





