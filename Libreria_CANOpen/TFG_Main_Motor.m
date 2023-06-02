% -----------------------------------------------------------------------------------------------------------------------
% @author: Cristian Jara Corros
% @NIA: 100440701
% @year: 2023
% @version: v1.0
% @TFGtitle: Librería de comunicación CANopen en MATLAB para un cuello robótico blando
% -----------------------------------------------------------------------------------------------------------------------

% -----------------------------------------------------------------------------------------------------------------------
% Ejemplo de uso utilizado para manipular un motor. En el se encuentra la
% configuración del motor y la orden para que se desplace a una posición.
% -----------------------------------------------------------------------------------------------------------------------

try
       canch=createChannelPEAKSystem(1000000);
       
       startChannelPEAKSystem (canch)
       obj = CiA402Device(1,canch);
        obj.initConfigurationData (2048, 24, 0.001, 0.144, 20);        % Parámetros para la utilización del cuello;
        obj.setupPositionMode (10);
catch
end

%--------------------- Vigilancia y activación del canal de comunicación
        pause(0.01); 
        obj.setPosition(1);   %Target position
        pause(0.01);      

        dato=obj.getStatusWord(); 
        obj.printStatusWord();
 
 
               