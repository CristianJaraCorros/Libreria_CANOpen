% -----------------------------------------------------------------------------------------------------------------------
% @author: Cristian Jara Corros
% @NIA: 100440701
% @year: 2023
% @version: v1.0
% @TFGtitle: Librería de comunicación CANopen en MATLAB para un cuello robótico blando
% -----------------------------------------------------------------------------------------------------------------------

% -----------------------------------------------------------------------------------------------------------------------
% · Creación del canal de comunicaciones del bus CAN (canch).
% · Creacción del objeto del elemento del cuello.
% · Creación del elemento sensor con los datos de comunicación necesarios.
% 
% Estas acciones se encuentra dentro de dos try-catch para que en el caso
% de que el usuario ejecute varias veces seguidas el programa no de fallo
% ya que estos se crean la primera vez que se inicia el programa.
% ----------------------------------------------------------------------------------------------------------------------- 
 try
       canch=createChannelPEAKSystem(1000000);
       cuello = Elemento(canch);
 catch
     %disp('Canal de comunicación y elemento ya creado o ha ocurrido un fallo');
 end

 try
       sen = IMU3DM_GX5_10('COM3',50);
 catch
     %disp('Elemento sensor ya creado o ha ocurrido un fallo');
 end
% -----------------------------------------------------------------------------------------------------------------------
% · Activación del canal de comunicaciones.
% · Creación y configuración de los drivers de la articulación.
% · Configuración del Elemento.
% ----------------------------------------------------------------------------------------------------------------------- 
    startChannelPEAKSystem (canch);

    cuello.ConfiguracionDriver([1 2 3]);                                % Direcciones del cuello 

    cuello.configurationKinematicsValues(0.045,0.045,0.125,0.0065);     % Parámetros para la utilización del cuello  
                                                                        % 0.045 -> Distancia del eje central a la periferia de conexión del motor
                                                                        % 0.045 -> Distancia del eje central a la periferia de conexión del motor
                                                                        % 0.125 -> Longitud de la articulación blanda
                                                                        % 0.0065 -> Radio del motor    
    cuello.initConfigurationData (2048, 24, 0.001, 0.144, 20);          % Parámetros para la utilización del cuello
                                                                        % 2048 -> Resolución del encoder incremental, en función del número de pulsos
                                                                        % 24 -> Ratio del desplazamiento del motor-carga
                                                                        % 0.001 -> Periodo de muestreo
                                                                        % 0.144-> Limite de corriente admitida por el motor
                                                                        % 20 -> Limite de corriente admitida por el driver



%     cuello.ConfiguracionDriver([31 32 33]);                             % Direcciones del brazo 
% 
%     cuello.configurationKinematicsValues(0.052,0.052,0.2,0.0093);      % Parámetros para la utilización del brazo
%     cuello.initConfigurationData (2048, 157, 0.001, 1.25, 20);         % Parámetros para la utilización del brazo

    cuello.busChangeVelocity(15);                                       % Cambiar la velocidad de control de los drivers
    cuello.setupPositionMode(15);                                        % Establecer el modo de funcionamiento: Modo posición

% -----------------------------------------------------------------------------------------------------------------------
% · Inicialización de las variables de consigna y registro de datos
% ----------------------------------------------------------------------------------------------------------------------- 
    incl=0;orient=0;

 %   cuello.busGetStatusWord();                    % Comprobación informativa de la palabra de estado

    DatosInclinacion=[];
    DatosOrientacion=[];
    Datos3D=[];

% -----------------------------------------------------------------------------------------------------------------------
% · Ejecución de movimientos y mediciones
% ----------------------------------------------------------------------------------------------------------------------- 
for i=1:130
    % Transformación de las consignas dadas en datos interpretables por los motores. Seguido del envío de la consigna.
    
    pos=cuello.GetIK(incl,orient);    
    cuello.busSetPosition(pos);

    for j=0:80  % Toma de datos del sensor hasta su estabilización
            dato=sen.GetPitchRollYaw();
    end

    % Toma del dato del sensor definitivo para la graficacion
        dato=sen.GetPitchRollYaw();
    
    % Guardar los parámetros para su posterior graficación.
    [inclinacion orientacion]=getInclinacionOrientacion(dato(1),dato(2));   % Transformación de la información recibida para que tenga el mismo formato que las consignas enviadas.
    DatosInclinacion=vertcat(DatosInclinacion,[inclinacion incl]);          % Guarda la consignas más el dato.
    DatosOrientacion=vertcat(DatosOrientacion,[orientacion orient]);        % Guarda la consignas más el dato.

    % Visualización de las consignas enviadas junto a los valores registrados por el sensor inercial.
    disp("Inclinacion: "+incl+"; "+inclinacion+"; Orientacion: "+orient+"; "+orientacion+";")


    dimension=cuello.getDimensionParameter();                               % Obtención de los parámetros para posteriormente generar los datos necesario para realizar la gráfica 3D.
    [x y z]=puntoDeEspacio(dimension(3),inclinacion,orientacion);
    Datos3D=vertcat(Datos3D,[x y z]);
  
        orient=orient+20;           % Actualización de consignas para el próximo ciclo
        if orient>360               % Cada vez que de una vuelta completa se inicia el giro en otra inclinación
            incl=incl+5;
            orient=0;
        end
end

% -----------------------------------------------------------------------------------------------------------------------
% · Parada de la comunicación y grafica 3D
% ----------------------------------------------------------------------------------------------------------------------- 
    stopChannelPEAKSystem (canch);


    x=Datos3D(:,1);
    y=Datos3D(:,2);
    z=Datos3D(:,3);
    p=plot3(x,y,z,'o');  % Gráfica en 3D
            
            figure  % Gráfica de inclinación de la articulación
            plot(DatosInclinacion)
            title(DatosInclinacion,'Inclinacion')
            xlabel('nº de muestra') 
            ylabel('grados') 
            legend('Registrado','Objetivo')
            
            figure  % Gráfica de orientación de la articulación
            plot(DatosOrientacion)
            title(DatosOrientacion,'Orientacion')
            xlabel('nº de muestra') 
            ylabel('grados') 
            legend('Registrado','Objetivo')
