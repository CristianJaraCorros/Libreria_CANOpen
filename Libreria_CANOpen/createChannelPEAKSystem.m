% -----------------------------------------------------------------------------------------------------------------------
% @author: Cristian Jara Corros
% @NIA: 100440701
% @year: 2023
% @version: v1.0
% @TFGtitle: Librería de comunicación CANopen en MATLAB para un cuello robótico blando
% -----------------------------------------------------------------------------------------------------------------------

% -----------------------------------------------------------------------------------------------------------------------
% createChannelPEAKSystem (BusSpeed): esta función crea y asigna a una variable el canal de
% comunicaciones, específico del adaptador PEAKSystem, que utilizará
% Matlab. Este canal será el utilizado para mantener la comunicación entre
% los drivers de los motores y el software de Matlab del ordenador.
%
% A la hora de crear el canal se debe indicar la misma velocidad de
% transmisión que está configurada en los drivers. Al ser una función
% específica para un adaptador en concreto, internamente, se configura el
% canChannel con los datos 'PEAK-System' y 'PCAN_USBBUS1', que hacen
% referencia al modelo y tipo de conexión del adaptador. Posteriormente se
% ajusta la velocidad a la indicada por el usuario con el método configBusSpeed, propio de Matlab.
% ----------------------------------------------------------------------------------------------------------------------- 
function Canal = createChannelPEAKSystem (BusSpeed)
    Canal = canChannel('PEAK-System','PCAN_USBBUS1'); 
    configBusSpeed(Canal,BusSpeed);
end