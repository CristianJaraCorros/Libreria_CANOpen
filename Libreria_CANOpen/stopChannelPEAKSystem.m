% -----------------------------------------------------------------------------------------------------------------------
% @author: Cristian Jara Corros
% @NIA: 100440701
% @year: 2023
% @version: v1.0
% @TFGtitle: Librería de comunicación CANopen en MATLAB para un cuello robótico blando
% -----------------------------------------------------------------------------------------------------------------------

% -----------------------------------------------------------------------------------------------------------------------
% stopChannelPEAKSystem (canch): función utilizada para detener la
% transmisión de datos del canal de comunicaciones. El usuario debe indicar el canal que quiere
% parar, una vez desactivado aparecerá el mensaje "Canal desactivado.". Sí
% ya estuviera desactivado será notificado mediante el aviso
% "Canal actualmente desactivado.". En el caso de que la variable introducida no se corresponda
% con un canal previamente creado, se mostrará el siguiente mensaje: "Canal
% no creado."
% ----------------------------------------------------------------------------------------------------------------------- 
function stopChannelPEAKSystem (canch)
    try
       if canch.Running == true
           stop(canch)
           disp('Canal desactivado.')
       else 
           disp('Canal actualmente desactivado.')
       end
    catch
        disp('Canal no creado.')
    end
end