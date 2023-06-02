% -----------------------------------------------------------------------------------------------------------------------
% @author: Cristian Jara Corros
% @NIA: 100440701
% @year: 2023
% @version: v1.0
% @TFGtitle: Librería de comunicación CANopen en MATLAB para un cuello robótico blando
% -----------------------------------------------------------------------------------------------------------------------

% -----------------------------------------------------------------------------------------------------------------------
% Elemento: es heredera de la clase 'handle' al igual que lo era
% CiA402DeviceDictionary y por el mismo motivo. El motivo por el cual se
% crea esta clase es para dotar a lo usuarios de un elemento de alto nivel
% por el cual pueda controlar un elemento compuesto de varios dispositivos
% conectados al bus de comunicaciones CANopen. De esta forma, se aleja al
% usuario de la capa que se relaciona de forma directa con el protocolo de
% comunicaciones, permitiéndole trabajar directamente comandando los
% motores mediante consignas de posición.
% ----------------------------------------------------------------------------------------------------------------------- 
classdef Elemento<handle

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Atributos
    properties 
        % Canal de comunicaciones por el que se realizará el envio y recepción de mensajes
            canch;
        % Atributos de las dimensiones del elemento a controlar
            a=0.035; %0.045;            % Distancia del centro de la extructura al perímetro de la articulación
            b=0.035; %0.045;            % Distancia del centro de la extructura al perímetro de la articulación
            L0=0.2; %0.116+0.09
            radio=0.0093; %0.0065
        % Matriz que engloba los drivers conectados al bus y que componen el elemento
            Driver;
    end

    %---------------------------------------------------------------------- Métodos
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Métodos constructivos y de configuracion
        % -----------------------------------------------------------------------------------------------------------------------
        % Elemento(new_canch): constructor de la clase de un elemento a
        % controlar. El usuario tendrá que indicar el canal por el cual se
        % realiza la comunicación con los nodos del sistema (canch).
        % ----------------------------------------------------------------------------------------------------------------------- 
        function obj = Elemento(new_canch)
            obj.canch=new_canch;
            obj.Driver=[];
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % ConfiguracionDriver(obj,identificadores): el usuario deberá
        % indicar, por medio de una matriz fila, el número de 
        % identificación de los equipos de los nodos conectados en la red
        % CANopen. 
        %
        % Una vez que el usuario haga uso de esta función se crearán los
        % objetos de la clase CiA402Device correspondientes a los motores
        % del elemento creado. Al mismo tiempo, se añade en el atributo
        % Driver del objeto de la clase elemento el número de los nodos creados.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function ConfiguracionDriver(obj,identificadores)
                obj.Driver=[];                                             % Inicialización de la matriz de drivers
                for i=1:length(identificadores)
                    newDriver=CiA402Device(identificadores(i),obj.canch);  % Creación del objeto del driver
                    obj.Driver = horzcat(obj.Driver,newDriver);            % Guardar número del nodo
                end
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % setupPositionMode(obj,velocity): método utilizado para configurar
        % el modo posición de los drivers que componen el elemento creado
        % de esta clase. Cuenta con un parámetro de entrada referente a la
        % velocidad. Esta velocidad será única y común a todos los motores,
        % rigiendo la rapidez con la que se realizarán los movimientos.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function setupPositionMode(obj,velocity)
                % Definir los nuevos valores de velocidad caracteristicos de los drivers utilizados para el funcionamiento de los motores            
                for i=1:length(obj.Driver)
                    obj.Driver(i).setupPositionMode(velocity);                    
                end
        end


        % -----------------------------------------------------------------------------------------------------------------------
        % getDimensionParameter(obj): devuelve una matriz con la
        % información relativa a los cuatro parámetros configurables para
        % el elemento en cuestión.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function dimension=getDimensionParameter(obj)
             dimension=[obj.a obj.b obj.L0 obj.radio];
             % disp('Las dimensiones configuradas en este elemento son: '+'a= '+obj.a+'; b= '+obj.b'+'; L0= '+obj.L0+'; radio= '+obj.radio+';')
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Métodos de ordenes de cambios de modo, operación y recepción de información concreta
        % -----------------------------------------------------------------------------------------------------------------------
        % busSetPosition(obj,consigna): envío de los nuevos datos de
        % posición a cada uno de los motores que componen el objeto creado.
        % En primer lugar, se comprobará que se hayan introducido tantas
        % consignas como motores se hayan configurado en el bus de
        % comunicaciones. De ser así, se enviarán las consignas a todos los
        % elementos del driver. En caso contrario no se realizará el envío
        % de los nuevos setpoints. 
        % ----------------------------------------------------------------------------------------------------------------------- 
        function busSetPosition(obj,consigna)
            ConsignasOk=false;
            % Comprobación de que haya un número de consignas tales que 
            % exista una para cada uno de los drivers especificados en el 
            % bus. En caso de que no se cumpla, no se realizará el
            % movimiento de los motores

	        if length(consigna)==(length(obj.Driver))
                ConsignasOk=true;
            else
                % disp('El número de consignas no es adecuada al número de drivers.'+length(consigna)+":"+length(obj.Driver))
                % disp('Asegurese de que por cada driver introduce una consigna de posición.')
            end
            % Escritura y envío de las ordenes de posición y velocidad
            if ConsignasOk==true
                for i=1:length(obj.Driver)                           % Envío de las consignas a cada motor            
                    obj.Driver(i).setPosition(consigna(i));          % Envío de las consigas de posición
                    pause(0.01);                                         
                end             
            end
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % busChangeVelocity(obj,consigna): el usuario, por medio de una
        % consigna, podrá definir la velocidad a la que los motores 
        % realicen sus movimientos.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function busChangeVelocity(obj,consigna)
                for i=1:length(obj.Driver)
                    obj.Driver(i).setVelocity(consigna);pause(0.01);          %Target speed
                end
        end
        
        % -----------------------------------------------------------------------------------------------------------------------
        % busGetStatusWord(obj): método utilizado para realizar la solicitud 
        % de la word de estado de todos los driver que conforman el 
        % elemento. Se realiza una consulta mediante el envio de un mensaje
        % solicitando la StatusWord y se imprime por pantalla el estado de 
        % cada uno de los controladores del elemento.
        % -----------------------------------------------------------------------------------------------------------------------    
        function status=busGetStatusWord(obj)        
            status=zeros(1,length(obj.Driver));
            for i=1:length(obj.Driver)
                dato=obj.Driver(i).getStatusWord();
                status(i)=bin2dec(dato);
                obj.Driver(i).printStatusWord();
                %pause(0.01);
            end
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % busGetTargetPosition(obj): este método realiza una consulta a
        % todos los controladores para conocer sí han alcanzado la posición
        % consignada. Se comprueba la palabra de estado y sí todos los
        % motores han alcanzado la posición la función devuelve un '1',
        % mientras que sí alguno no ha logrado alcanzar la posición se
        % obtendrá un '0'.
        % -----------------------------------------------------------------------------------------------------------------------    
        function status=busGetTargetPosition(obj)        
            %Target=zeros(1,length(obj.Driver));
            numTargetReach=0;
            status=0;
            % Escritura y envío de las ordenes de posición y velocidad
            for i=1:length(obj.Driver)
                TargeT_Alcanzada=obj.Driver(i).getStatusWord();
                %pause(0.01);
                mask= dec2bin(hex2dec("0400"),16);                             % Escritura de una máscara que será aplicada a la consigna tratada anteriormente
                Dato=bitand(bin2dec(TargeT_Alcanzada),bin2dec(mask));                 % Comparacion de bit con la mascara para filtrar el dato y montar la matriz correcta

                    switch Dato             % Elección del mensaje correcto
                       case 0x400
                            %disp("Driver: "+obj.Driver(i).getId()+" Alcanzada la posición")
                            numTargetReach=numTargetReach+1;
                        otherwise
                            %disp("Driver: "+obj.Driver(i).getId()+" No han alcanzado todos la posición")
                    end
            end
            
            if numTargetReach == length(obj.Driver)
                           disp('Todos los motores han alcanzado la posición')
                status=1;
            end            
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % cambioSentidoGiro(obj,cambioSentido):por medio de este método se
        % modifica el sentido de giro de todos los motores que componen el
        % elemento. El usuario introducirá una matriz en la que indicará con
        % un '1' aquellos controladores que cambien el sentido de giro y
        % con un '0' aquellos que no quiera cambiar. Sí las consignas
        % introducidas por el usuario no se corresponde con el número de
        % controladores, se generará un mensaje de advertencia.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function cambioSentidoGiro(obj,cambioSentido)
            if length(cambioSentido) == length(obj.Driver)
                for i=1:length(obj.Driver) % Por medio del bucle for se cambiará el sentido de gira de cada uno de los motores.
                    if cambioSentido(i)==1 
                        obj.Driver(i).cambioSentidoGiro();
                    end
                end
            else
                disp('El nº de consignas no se corresponde con la cantidad de drivers del elemento')
            end
        end
        
        % -----------------------------------------------------------------------------------------------------------------------
        % busGetPosition(obj): función utiilzada para realizar la consulta
        % de la posición de las posición actual de todos los motores que
        % componen el elemento. Se devuelve una matriz fila con tantas
        % columnas como drivers componen el elemento.
        % -----------------------------------------------------------------------------------------------------------------------    
        function posicion=busGetPosition(obj)        
            posicion=zeros(1,length(obj.Driver));
            % Escritura y envío de las ordenes de posición y velocidad
            for i=1:length(obj.Driver)
                posicion(i)=obj.Driver(i).getPosition();
                %pause(0.01);
            end
        end

        % -----------------------------------------------------------------------------------------------------------------------
        %  initConfigurationData (obj,encoder, ratio, sample,
        %  limitCurrentMotor, limitCurrentDriver): en este método se
        %  definen los valores característicos de los drivers utilizados
        %  para el correcto funcionamiento de los motores. Los valores
        %  indicados serán utilizados para el correcto escalado de las
        %  consignas introducidas por el usuario.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function initConfigurationData (obj,encoder, ratio, sample, limitCurrentMotor, limitCurrentDriver)    
            % Definir los nuevos valores caracteristicos de los drivers utilizados para
            % el funcionamiento de los motores            
                for i=1:length(obj.Driver)
                    obj.Driver(i).initConfigurationData (encoder, ratio, sample, limitCurrentMotor, limitCurrentDriver);                    
                end
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % getDriver(obj,ID): en caso de que el usuario desee utilizar uno
        % de los drivers de forma independiente, este podrá solicitar el
        % objeto que está asociado a un identificador ID concreto. El
        % método devolverá el objeto solicitado.
        % -----------------------------------------------------------------------------------------------------------------------    
        function driver=getDriver(obj,ID)        
            for i=1:length(obj.Driver)                                      % Búsqueda del identificador entre los elementos que componen el elemento
                if obj.Driver(i).getId()==ID
                   driver=obj.Driver(i);                                    % Devolución del objeto
                   return
                end
            end
            driver=null;                                                    % En el caso de que el identificador no exista, no se enviará nada
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Métodos cinemáticos
        % -----------------------------------------------------------------------------------------------------------------------
        % configurationKinematicsValues(obj,new_a,new_b,new_L0,new_radio): 
        % por medio de este método se realiza la configuración de los
        % parámetros dimensionales del elemento a controlar.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function configurationKinematicsValues(obj,new_a,new_b,new_L0,new_radio)
            obj.a=new_a;                        % Distancia del eje central a la periferia de conexión del motor
            obj.b=new_b;                        % Distancia del eje central a la periferia de conexión del motor
            obj.L0=new_L0;                      % Longitud de la articulación blanda
            obj.radio=new_radio;                % Radio del motor
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % GetIK(obj,inclinacion,orientacion): por medio de las consignas de
        % inclinación y orientación introducidas por el usuario, este
        % método devolverá la posición en la que cada motor se debe
        % colocar.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function Theta=GetIK(obj,inclinacion,orientacion)    
            %--------------------- Obtención de los ángulos según los datos de entrada        
            theta=deg2rad(inclinacion);
            phi=deg2rad(orientacion);        
            %--------------------- Obtención de calculos        
            if theta~=0                 % Es necesario ajustar los valor de phix con relación a la posición del sensor y los motores.
                R=obj.L0/theta;   
                % Datos específicos para el cuello robótico actualmente 2023/05/17
                phi1= -phi+pi;          %pi/2;
                phi2= -phi+5*pi/3;      %7*pi/6;
                phi3= -phi+pi/3;        %11*pi/6;  

                %--------------------- Escritura de los resultados de las operaciones para el cuello
                Longitud(1)=(2*R - 2 * obj.a * cos(phi1))*sin(theta/2);
                Longitud(2)=(2*R - 2 * obj.a * cos(phi2))*sin(theta/2);
                Longitud(3)=(2*R - 2 * obj.a * cos(phi3))*sin(theta/2);

            else
                %--------------------- Escritura de los resultados de las operaciones
                Longitud(1)=obj.L0;
                Longitud(2)=obj.L0;
                Longitud(3)=obj.L0;
            end     
                    %----------------------------------- Theta para el cuello robótico
                    Theta(1)=(obj.L0-Longitud(1))/obj.radio;
                    Theta(2)=(obj.L0-Longitud(2))/obj.radio;
                    Theta(3)=(obj.L0-Longitud(3))/obj.radio;   
            %disp("Las consignas de posicion seran: "+Longitud(1)+"|"+Longitud(2)+"|"+Longitud(3))
        end
   
    % Funciones no implementadas
   

    %         % -----------------------------------------------------------------------------------------------------------------------
    %         %  Si las consignas (pos1,vel1,pos2,vel2...) se queda como esta
    %         %  Si las consignas (pos,vel) la consigna de SetVelocity se sustituye j ->length(Driver)+j
    %         % FALTA POR PROBAR
    %         % ----------------------------------------------------------------------------------------------------------------------- 
    %         function busSetVelocity(obj,consigna)
    %             ConsignasOk=false;
    %             j=1;         % Indice utilizado para llamar a las consignas de posicion y velocidad 
    %             % Comprobación de que haya un número de consignas pares para cada uno
    %             % de los drivers especificados en el bus
    % 		        if(mod(length(consigna),2)==0) && length(consigna)==(2*length(obj.Driver))
    %                     ConsignasOk=true;
    %                 else
    %                     % disp('El número de consignas no es adecuada al número de drivers.')
    %                     % disp('Asegurese de que por cada driver introduce una consigna de posición y otra de velocidad.')
    %                 end
    %             % Escritura y envío de las ordenes de posición y velocidad
    %             if ConsignasOk==true
    %                 for i=1:length(obj.Driver)    
    %                     obj.Driver(i).startMotor();
    %                     obj.Driver(i).modoOperacion()
    %                     pause(0.01);    
    %                     obj.Driver(i).setPosition(consigna(j));pause(0.01);          %Target position
    %                     j=j+1;
    %                     obj.Driver(i).setVelocity(consigna(j));pause(0.01);          %Target speed
    %                 end
    %              
    %                 for j=1:length(obj.Driver)    
    %                     obj.Driver(j).rofileStar();                    %Star profile
    %                     pause(0.01);                                    
    %                 end
    %             end
    %         end
    end
end