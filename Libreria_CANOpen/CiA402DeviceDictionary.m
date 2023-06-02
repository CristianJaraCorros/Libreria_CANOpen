% -----------------------------------------------------------------------------------------------------------------------
% @author: Cristian Jara Corros
% @NIA: 100440701
% @year: 2023
% @version: v1.0
% @TFGtitle: Librería de comunicación CANopen en MATLAB para un cuello robótico blando
% -----------------------------------------------------------------------------------------------------------------------

% -----------------------------------------------------------------------------------------------------------------------
% CiA402DeviceDictionary: Esta clase tiene el único propósito de funcionar 
% como una librería en la que se guardan los registros de los mensajes a 
% los que se puede acceder por comunicaciones. Por ello, esta clase 
% únicamente cuenta con un constructor vacío para crear el objeto 
% obj = CiA402DeviceDictionary()". Los registros se han guardado en 
% atributos a los que el usuario no tendrá acceso directo ya que para ello 
% tendrá los métodos dedicados en las clases hijas. 
%          
% Por otro lado, está clase hereda de la clase 'handle' de Matlab.Es 
% necesario realizar está herencia para que el sistema de clases funcione y 
% no se consideré como funciones abstractas. 
% ----------------------------------------------------------------------------------------------------------------------- 
classdef CiA402DeviceDictionary<handle

    %---------------------------------------------------------------------- Atributos
    properties (Access = protected)
        targetPosition = [0x7A 0x60 0];         % Dirección del Target de posición 607Ah 0h 
        targetVelocidad = [0x81 0x60 0];        % Dirección del Target de velocidad 6081h 0h 
        targetAceleration = [0x83 0x60 0];      % Dirección del Target de aceleracion 6083h 0h 
        OperationMode = [0x60 0x60 0];          % Dirección del Modo operacion 6060h 0h 
        positionmode = [1];                     % Setup del modo de funcionamiento: posición 1h
        
        controlword=[0x40 0x60 0];              % Dirección de la word de control 6040h 0h
        statusword=[0x41 0x60 0];               % Dirección de la word de estado 6041h 0h
        
        readySwitchOn=[6 0];                    % Orden para ordenar que el Switch este listo 0006h
        switchOn=[7 0];                         % Orden de activación del Switch 00007h
        goswitchondisable=[0 0];                % Orden de apagado del Switch 0000h
        enable=[0x0F 0];                        % Orden de habilitación de las operaciones 000Fh
        starProfile=[0x1F 0];                   % Inicio de la acción del driver para que se situe en la posición indicada 001Fh
        runViaControlWord=[0x3F 0]              % Inicio de la acción del driver utilizando la ControlWord (NO UTILIZADO)
        quickstop=[2 0];                        % Orden de parada de emergencia 0002h
        
        OperationModeDisplay=[0x61 0x60 0];     % Dirección de la petición de información de modo de operación activo 6061h 0h
        
        velocitymode=[3];                       % Setup del modo de funcionamiento: velocidad 3h (NO UTILIZADO)
        
        quick_stop_mode=[0x5A 0x60 0];          % Dirección del modo de parada rápida (NO UTILIZADO)
        stop_option_code=[0x5D 0x60 0];         % Comando para la detención de operación (NO UTILIZADO)
        
        getCurrentPosition=[0x64 0x60 0];       % Dirección de la petición de información de la posición actual 6064h 0h
        getCurrentVelocity=[0x69 0x60 0];       % Dirección de la petición de información de la velocidad actual 6069h 0h (NO UTILIZADO)
        getCurrentVelocityDemand=[0x6B 0x60 0]; % Dirección de la petición de información de la velocidad de demanda 606Bh 0h (NO UTILIZADO)
        
        resetPosition=[0x0F 0];                 % Dirección del reset de posición 000Fh
        getDemandPosition=[0x62 0x60 0];        % Dirección de la petición de información de la posición de demanda 6062h 0h
    end

    %---------------------------------------------------------------------- Métodos
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Constructores y métodos de configuración
        % -----------------------------------------------------------------------------------------------------------------------
        % obj = CiA402DeviceDictionary(): constructor vacío de la clase.
        % -----------------------------------------------------------------------------------------------------------------------
        function obj = CiA402DeviceDictionary()
            
        end
    end
end