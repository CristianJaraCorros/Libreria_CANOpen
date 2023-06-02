% -----------------------------------------------------------------------------------------------------------------------
% @author: Cristian Jara Corros
% @NIA: 100440701
% @year: 2023
% @version: v1.0
% @TFGtitle: Librería de comunicación CANopen en MATLAB para un cuello robótico blando
% -----------------------------------------------------------------------------------------------------------------------

% -----------------------------------------------------------------------------------------------------------------------
% startChannelPEAKSystem (canch): función utilizada para inicializar la transmisión de datos del
% canal de comunicaciones. Tras crear el canal de comunicaciones se deberá
% realizar la llamada a esta función para abrir el canal de comunicación
% con los nodos del sistema. El usuario debe indicar el canal que quiere
% inicializar, una vez activo aparecerá el mensaje "Canal activado.". Sí
% ya estuviera previamente activado será notificado mediante el aviso
% "Canal actualmente activado.". En el caso de que la variable introducida no se corresponda
% con un canal previamente creado, se mostrará el siguiente mensaje: "Canal
% no creado."
% ----------------------------------------------------------------------------------------------------------------------- 
function startChannelPEAKSystem (canch)
    try
       if canch.Running == false
           start(canch)
           disp('Canal activado.')
       else 
           disp('Canal actualmente activado.')
       end
    catch
        disp('Canal no creado.')
    end
end