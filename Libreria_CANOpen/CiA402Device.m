% -----------------------------------------------------------------------------------------------------------------------
% @author: Cristian Jara Corros
% @NIA: 100440701
% @year: 2023
% @version: v1.0
% @TFGtitle: Librería de comunicación CANopen en MATLAB para un cuello robótico blando
% -----------------------------------------------------------------------------------------------------------------------

% -----------------------------------------------------------------------------------------------------------------------
% CiA402Device: clase que hereda de CiA402DeviceDictionary. El objetivo de
% esta clase es la de proporcionar al usuario las herramientas necesarias
% para poder interactuar con los drivers.
% ----------------------------------------------------------------------------------------------------------------------- 
classdef CiA402Device <CiA402DeviceDictionary
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Atributos
    properties  (Access = private)
        % Canal de comunicaciones por el que se realizará la transmisión de mensajes
            canch;
        % Atributos propios del dispositivo y asignación de valores iniciales.
            id=0;
            controlWord=0;
            statusWord='0';
            actargetPosition=0;
            actargetVelocity=10;
            actualPosition=0;
            actualVelocity=0;
        % Atributos utilizados para el escalado de las consignas de velocidad y posición
            encoderResolucion=0;            % Resolución del encoder incremental, en función del número de pulsos
            mlRatio=0;                      % Ratio del desplazamiento del motor-carga
            SampleSL=0;                     % Periodo de muestreo
            LimiteCorrienteMotor=0;         % Limite de corriente admitida por el motor
            LimiteCorrienteDriver=0;        % Limite de corriente admitida por el driver
            SentidoGiro=1;                  % Se puede modificar el sentido de giro via software, en el caso de que al mandar una consigna el motor gire en dirección contraria. SentidoGiro==1 giro esperado; SentidoGiro=-1 giro en sentido contrario al esperado
    end

    %---------------------------------------------------------------------- Métodos
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Constructores y métodos de configuración
        % -----------------------------------------------------------------------------------------------------------------------
        % obj = CiA402Device(inputId,inputCanch): constructor de la clase
        % del driver de cada motor que componen una articulación. El
        % usuario tendrá que indicar el Id o dirección del nodo del
        % dispositivo y el canal por el cual se realizará la comunicación
        % (canch). Tras esto, se realizará una llamada al método
        % "startMotor" para indicar al driver que será comandado de forma
        % remoto y que debe activar su switch interno.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function obj = CiA402Device(inputId,inputCanch)
            obj.id=inputId;                         % Asignación de la Id de entrada al atributo id del objeto creado
            obj.canch=inputCanch;                   % Asignación del canal de comunicación al atributo canch del objeto creado
            obj.startMotor();                       % Llamada al método para dejar preparado al motor
            %pause(0.01);
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % initConfigurationData (obj,encoder, ratio, sample,
        % limitCurrentMotor, limitCurrentDriver): todas las
        % operaciones de escalo/desescalado de las consignas enviadas a los
        % motores deben ser tratadas previamente. 
        % 
        % Es necesario especificar una serie de parámetros previamente. Por
        % lo que el usuario podrá utilizar este método para definir los 
        % valores cada uno de los atributos: resolución del encoder 
        % (encoder), ratio de desplazamiento (ratio), periodo de muestreo 
        % (sample), límite de corriente del motor (limitCurrentMotor) y 
        % límite de corriente del driver (limitCurrentDriver). Todos los 
        % atributos deben ser no nulos cuando se realiza la llamada al 
        % método.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function initConfigurationData (obj,encoder, ratio, sample, limitCurrentMotor, limitCurrentDriver)    
            obj.encoderResolucion=encoder;                  % Resolución del encoder incremental, en función del número de pulsos
            obj.mlRatio=ratio;                              % Ratio del desplazamiento del motor-carga
            obj.SampleSL=sample;                            % Periodo de muestreo
            obj.LimiteCorrienteMotor=limitCurrentMotor;     % Limite de corriente admitida por el motor
            obj.LimiteCorrienteDriver=limitCurrentDriver;   % Limite de corriente admitida por el driver
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % cambioSentidoGiro(obj): método muy sencillo que modifica el
        % atributo de sentido de giro, para realizar 
        % ----------------------------------------------------------------------------------------------------------------------- 
        function cambioSentidoGiro(obj)
                    obj.SentidoGiro=-1*obj.SentidoGiro;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Métodos de lectura y escritura generales
        % -----------------------------------------------------------------------------------------------------------------------
        % writeSDO (obj,direccion, comando): escribe un comando de
        % distintas longitudes en una dirección especifica del dispositivo. Para
        % ello se utilizan como parámetros de entra la dirección del objeto
        % de dirección y el comando con la orden requerida.
        % 
        % En este método se ejecutan 5 pasos antes de poder enviar el
        % mensaje: 
        % - En primer lugar se debe comprobar el tamaño que tendrá
        % el mensaje que se va a enviar para inicializar una matriz fila 
        % que contendrá el mensaje a transmitir. Se sumará las longitudes de la
        % entrada dirección, la de comando y se añadirá una posición extra.
        % Esto se hace así para respetar la estructura de un mensaje SDO en
        % CANOpen. La longitud de la dirección aporta el tamaño de bytes
        % del índice y del subindice. La entrada de comando indica la
        % longitud de bytes de datos y por último el valor unitario extra
        % hace referencia al tamaño de datos.
        % - Se consulta la longitud del comando para definir la primera 
        % posición de la matriz con el tamaño del dato que se va a 
        % transmitir: 0 Bytes(40h), 8 Bytes(2Fh), 16 Bytes(2Bh) o 32 Bytes(23h).
        % - Se añade a la matriz la información del índice y del subíndice.
        % - Se añade toda la información referente al comando introducido.
        % Una vez este completo el mensaje se envía especificando que el
        % COB-ID será el 600 (el característico de los SDO).
        % ----------------------------------------------------------------------------------------------------------------------- 
        function writeSDO (obj,direccion, comando)
            longitudMensaje=1+length(direccion)+length(comando);     % Definir la longitud del mensaje que se debe escribir
            DATO = zeros(1,longitudMensaje);                         % Crear matriz de ceros sobre la que se va a escribir el mensaje

            switch length(comando)              % Definir el tamaño del dato a enviar
               case 0
                  DATO(1)=0x40;                 % Tamaño de datos: 0 Bytes
               case 1
                  DATO(1)=0x2F;                 % Tamaño de datos: 8 Bytes
               case 2
                  DATO(1)=0x2B;                 % Tamaño de datos: 16 Bytes
               otherwise
                  DATO(1)=0x23;                 % Tamaño de datos: 32 Bytes
            end
        
            for i = 1:length(direccion)         % Añadir la información del mensaje        
                DATO(1+i)= direccion(i);               
            end
            
            for i = 1:length(comando)           % Añadir el comando al mensaje
                DATO(4+i)= comando(i);               
            end           
        
            obj.PutMsg(600,DATO);               % Método de transmisión del mensaje por el bus.            
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % writePDO1 (obj,comando): por medio de las PDO se mandan mensajes
        % a objetos de dirección específicos de cada driver. Al utilizar la
        % PDO1 se enviará al COB-ID 200 el dato introducido en en el
        % parámetro de entrada 'comando'.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function writePDO1 (obj,comando)
            obj.PutMsg(200,comando);                   % Envío del mensaje por el bus.        
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % writeNMT (obj,comando): utiliza el servicio de red (NMT: network
        % managment) para cambiar los estados del dispositivo de destino y
        % pasar de un NMT a otro. El mensaje NMT consta de una trama de 2
        % bytes en el que el primer byte hace referencia al nuevo estado y
        % el segundo al ID del nodo. Por ejemplo, se utiliza este mensaje
        % para iniciar el nodo en remoto e inicializa la comunicación con
        % este. 
        %
        % Se diferencia de los intercambios PDO en que estos no necesitan
        % acceder a un registro concreto en el que se especifique el registro
        %+ID (COB-ID) ya que se manda un mensaje a la red general y el
        %segundo byte del comando indica el nodo afectado.
        % -----------------------------------------------------------------------------------------------------------------------        
        function writeNMT (obj,comando)
            sz = length(comando);                               % Definir el tamaño del comando
              %xID = 0;                                          
            messageout = canMessage(0,false,sz);                % Creación el tipo CAN de mensaje de envio.Al ser un mensaje general se indica que el ID seré no especificado (0)
            DATO = zeros(1,sz);                                 % Generación de la matriz del comando                                
            for i = 1:sz                                        % Escritura del comando en la matriz
                str = string(comando(i));
                DATO(i)= str2double(str);                       % Los datos de envio se deben escribir como valores enteros ya que la librería de Matlab así lo demanda               
            end
            messageout.Data = DATO;                             % Asignación del mensaje generado al mensaje que se va a enviar por comunicaciones                             
            transmit(obj.canch,messageout);                     % Transmisión del mensaje con un método de la librería de Matlab
            %pause(0.1)
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % readSDO (obj,comando): método utilizado para realizar la
        % lectura de los datos guardados en un registro del diccionario de
        % objetos concreto. Para ello se especificará el registro que se
        % quiere consultar, para que el driver envíe los datos solicitados.
        % El mensaje tiene la misma longitud que cuando se escribe en una
        % SDO, en la que la transmisión será: Primer byte indica el tamaño
        % de datos (0 bytes: 40h), 3 bytes para el registro y 4 bytes
        % rellenados con valor 00h ya que se solicita información. 
        % ----------------------------------------------------------------------------------------------------------------------- 
        function readSDO (obj,comando)        
            longitudMensaje=1+length(comando)+4;    % La longitud del mensaje total que tendrá el mensaje
            DATO = zeros(1,longitudMensaje);        % Crear matriz de ceros sobre la que se va a escribir el mensaje
                  
            DATO(1)=0x40;                           % Definir el tamaño del dato a enviar. Tamaño de datos: 0 Bytes        
        
            for i = 1:length(comando)               % Añadir la información del mensaje        
                DATO(1+i)= comando(i);               
            end  
        
            for i = 1:4                             % Añadir el comando al mensaje
                DATO(4+i)= 0;               
            end
        
            obj.PutMsg(600,DATO);                   % Método de transmisión del mensaje por el bus.
            
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % PutMsg (obj,COBID,data): este es el método utilizado para
        % realizar la transmisión de los SDO y PDO. Como entrada, se
        % indicará el COB-ID y la cadena que se quiere transmitir. 
        % 
        % La función combinará el COB-ID introducido como entrada y el ID del
        % driver que esta guardado en el atributo id del objeto. Generará
        % la estructura del mensaje y se transmitirá por el bus para que el
        % nodo al que se le solicite la información genere una respuesta.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function PutMsg (obj,COBID,data)
            sz = length(data);                                  % Definir el tamaño del comando
            xCOBID = hex2dec(string(COBID));                    % Convertir el COB-ID de hexadecimal a valor decimal.
            xID = xCOBID+obj.id;

            messageout = canMessage(xID,false,sz);              % Creación el tipo CAN de mensaje de envio.Al ser un mensaje general se indica que el ID seré no especificado (0)    
            DATO = zeros(1,sz);                                 % Generación de la matriz del comando
            
            for i = 1:sz                                        % Escritura del comando en la matriz
                str = string(data(i));
                DATO(i)= str2double(str);                       % Los datos de envio se deben escribir como valores enteros ya que la librería de Matlab así lo demanda                  
            end
            
            messageout.Data = DATO;                             % Asignación del mensaje generado al mensaje que se va a enviar por comunicaciones        
            transmit(obj.canch,messageout);                     % Transmisión del mensaje con un método de la librería de Matlab
            %pause(0.1);
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % messagein = getMessage(obj): recoge todo los mensajes que se
        % encuentran en el buffer de entrada del canal de comunicación y
        % devuelve una matriz con todos los mensajes desglosados para su
        % posterior consulta.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function messagein = getMessage(obj) 
                   msgIn = receive(obj.canch,inf);      % Recibe todos los mensajes del buffer
                   messagein=obj.desgloseMsg(msgIn);    % Se envía al método de desglose para transformar los mensajes recibidos en una matriz que facilitrará la consulta de estos.
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % msg=desgloseMsg(obj,messagein): recibe como entrada todos los
        % mensajes del buffer de entrada del canal de comunicaciones. Una
        % vez recibido estos mensajes se comprueba la cantidad de mensajes
        % que se han recibido y se genera una matriz. Dicha matriz constará de 8
        % columnas y tantas filas como mensajes recibidos. La primera
        % columna guardará el COB-ID de cada mensaje mientras que las 7
        % restantes guardan los datos.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function msg=desgloseMsg(obj,messagein)
            i=length(messagein);
            msg=zeros(i,8);
            for j=1:i
                msg(j,1)=messagein(1,j).ID;
                for X=1:messagein(1, j).Length
                    msg(j,1+X)=messagein(1,j).Data(X);
                end        
            end   
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Métodos de ordenes de cambios de modo, operación y recepción de información concreta
        % -----------------------------------------------------------------------------------------------------------------------
        % startMotor (obj): método utilizado durante la construcción del
        % objeto de la clase CiA402Device, para indicar al driver que debe
        % pasar al modo de arranque remoto. Para ello realiza las llamadas
        % a los métodos writeNMT y writePDO1. Con el segundo método se
        % envía los mensajes para activar el switch del driver.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function startMotor (obj)           
            obj.writeNMT ([1 obj.id])           % Start remote mode: Envio de mensaje general a la linea. Para que solo se active aquel configurado con el "id"
            %pause(0.1)            
            obj.writePDO1 (obj.readySwitchOn)   % READY SWITCH ON: Para toda esta secuencia de mensajes se utiliza la PDO1(recibir) -> COB-ID=200+ID. Todos poseen el mismo tamaño en bytes
            %pause(0.1)            
            obj.writePDO1 (obj.switchOn)        % SWITCH ON: Activar el Switch del driver. Para parar el motor es necesario mandar un [0 0].
            %pause(0.1) 
        end
       
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Métodos de ordenes de cambios de modo, operación y recepción de información concreta
        % -----------------------------------------------------------------------------------------------------------------------
        % resetSetPoint (obj): método utilizado para reiniciar el valor del
        % setpoint de posición por la PDO1 (COB-ID=200+ID)
        % ----------------------------------------------------------------------------------------------------------------------- 
        function resetSetPoint (obj)
            obj.writePDO1 (obj.resetPosition)
            %pause(0.1)
        end 
        
        % -----------------------------------------------------------------------------------------------------------------------
        % setVelocity(obj,consigna): esta función es utilizada para definir
        % la velocidad a la que el driver comandará el movimiento de los
        % motores. El usuario introducirá la velocidad que desee e rad/s y
        % este método realizará un escalado de la velocidad para que se
        % ajuste a las dimensiones del motor. Tras esto, se construirá el
        % mensaje, se adaptará la consigna del formato de unit_32 a una matriz lineal de 4 bytes. La velocidad mínima a la
        % que se deben mover los motores es de 65536(0x10000) que hace
        % referencia a un 1 incremento por sampleo.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function setVelocity(obj,consigna)
            velocidad=consigna*65536*obj.ScallingFactorVelocity();  % La consigna introducida se escalará con el factor de velocidad
            
            % Se comprueba los que los valores de velocidad escalada sean válidos
            if(velocidad==0)                                        
                 velocidad=0;
            elseif(abs(velocidad)<1)
                 velocidad=65536*(velocidad/abs(velocidad));        % La velocidad mínima a la que se deben mover los motores es de 65536(0x10000) que hace referencia a un 1 incremento por sampleo
            elseif(abs(velocidad)<65536)
                 velocidad=65536*velocidad;
            end
        
            data = obj.Data32to4x8(velocidad);                      % Se transforma el valor de velocidad al formato de 4 bytes
            sz = length(data);
            comando=zeros(1,sz);                                    % Generación de la matriz del comando
            for i = 1:sz                                            % Transformar los datos a decimal
                str = string(data(i));
                comando(i)= str2double(str); 
            end
            obj.actargetVelocity=comando;                           % Guardar el valor de la velocidad objetivo en el atributo propio del objeto       
            obj.writeSDO (obj.targetVelocidad, comando)             % Transmitir el comando de velocidad a la SDO específica
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % setPosition(obj,consigna): el usuario hará uso de este método
        % para indicar la posición que el driver debe alcanzar.
        % Internamente, el método mandará al driver un mensaje para que
        % habilite el movimiento de la futura consigna de posición. Trás
        % esto, tomará el dato del usuario, lo escalará y lo tranformará en
        % una matriz de 4 bytes. Guardará el dato en un atributo del objeto
        % y enviará dos mensajes: uno para indicar la nueva posición y otro
        % para iniciar el movimiento.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function setPosition(obj,consigna)

            obj.writePDO1 (obj.enable)                                                          % Activar la disponibilidad del movimiento al futuro setpoint
            %pause(0.1)

            data = obj.Data32to4x8(obj.SentidoGiro*consigna*obj.ScallingFactorPosition());      % Escalado de la consigna y transformación de dato entero a matriz de 4 bytes
            sz = length(data);
            comando=zeros(1,sz);
            for i = 1:sz                                                                        % Escritura de los datos en el mensaje de comando
                str = string(data(i));
                comando(i)= str2double(str); 
            end
            obj.actargetPosition=comando;                                                       % Guardar el comando en un atributo del objeto
            obj.writeSDO (obj.targetPosition, comando)                                          % Envío de la posición que debe alcanzar el driver.
            obj.writePDO1 (obj.runViaControlWord)                                               % Arrancar la acción de alcanzar la posición consignada via ControlWord
            %pause(0.1)
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % modoOperacion (obj, mode): esta función es empleada para enviar al 
        % driver el modo de funcionamiento que debe aplicar para alcanzar 
        % las consignas dadas.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function modoOperacion (obj, mode)
           obj.writeSDO (obj.OperationMode, mode)    
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % setupPositionMode (obj, velocity): método para configurar el
        % funcionamiento del driver en modo posición y envío de la consigna
        % de la velocidad a la que realizará los movimientos.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function setupPositionMode (obj, velocity)
           obj.modoOperacion(obj.positionmode);
           obj.setVelocity(velocity);    
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % getPosition(obj): solicitud del dato de posición en el que se
        % encuentra el driver en el momento de realizar la consulta. El
        % método enviará un mensaje solicitando la posición actual y
        % analizará todos los mensajes del buffer de comunicación,
        % devolviendo el dato de la posición actual.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function dato=getPosition(obj)
            lecturaOk=false;                        % Marca para comprobar que se realiza la lectura de la posición del driver
            obj.readSDO (obj.getCurrentPosition);           % Envío de la solicitud para recibir la posición actual
            pause(0.01)
            
            
            msg=obj.getMessage();                           % Recepción de todos los mensajes del buffer
            [numRows,numCols] = size(msg);                  % Extrae el tamaño de la matriz ya que cada fila es un mensaje
            
            dato=0;
            try
                for i = 1:numRows                           % Bucle que pasa por cada fila para comprobar luego comprobar el mensaje
                    COBID = msg(i)-obj.id;                  % Al COBID se le sustrae el valor del ID del nodo, de esta forma solo se consultarán los mensajes cuyo COBID se 580h
                    IndexSubindex = msg(i,3:5);%[msg(i,3) msg(i,4) msg(i,5)];%
                    ValueRead = msg(i,6:9);%[msg(i,6) msg(i,7) msg(i,8) msg(i,9)];%
                    if(COBID==str2double(string(0x580)))                    % Tras quitar el valor del nodo se comprueba que el mensaje recibido sea del tipo correcto.
                        if(IndexSubindex==obj.getCurrentPosition)           % Comprobación de que el mensaje recibido es del objeto del driver requerido 6064h 0h
                            consigna=obj.Data4x8to32(ValueRead);                            % Tansformación de la matriz en un número
                            dato=obj.SentidoGiro*obj.DescallingFactorPosition(consigna);    % Desescalado de la velocidad
                            lecturaOk=true;
                        end
                    end 
                end 
            catch 
                lecturaOk=false;
            end
                                
            if lecturaOk==false                      % En caso de fallo se envía un mensaje a la consola
                disp("No se pudo leer la posición del driver:"+obj.id);     
            else
                obj.actualPosition=dato;                    % Se guarda el valor de la posición actual
            end
            dato=obj.actualPosition;                        % Se devuelve el dato de la posición guardado en el atributo del objeto
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % getId(obj): devuelve la dirección del nodo guardado en el
        % atributo del objeto.
        % -----------------------------------------------------------------------------------------------------------------------
        function direccion=getId(obj)
           %disp("Su identificador es "+obj.id)
           direccion=obj.id;
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % getStatusWord(obj): solicitud de la word de estado del driver
        % en el momento de realizar la consulta. El
        % método enviará un mensaje solicitando la StatusWord y
        % analizará todos los mensajes del buffer de comunicación,
        % devolviendo el dato de la palabra de estado en formato binario.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function dato=getStatusWord(obj)
            lecturaOk=false;                        % Marca para comprobar que se realiza la lectura del del driver 
            obj.readSDO (obj.statusword);                   % Envío de la solicitud para recibir la StatusWord
            pause(0.01)
            
            msg=obj.getMessage();                           % Recepción de todos los mensajes del buffer
            [numRows,numCols] = size(msg);                  % Extrae el tamaño de la matriz ya que cada fila es un mensaje
            
            dato=0;
            try
                for i = 1:numRows                           % Bucle que pasa por cada fila para comprobar luego comprobar el mensaje
                    COBID = msg(i)-obj.id;                  % Al COBID se le sustrae el valor del ID del nodo, de esta forma solo se consultarán los mensajes cuyo COBID se 580h
                    IndexSubindex = msg(i,3:5);%[msg(i,3) msg(i,4) msg(i,5)];%msg(i,3:5);%
                    ValueRead = msg(i,6:9);%[msg(i,6) msg(i,7) msg(i,8) msg(i,9)];%msg(i,6:9);%
                    if(COBID==str2double(string(0x580)))                    % Tras quitar el valor del nodo se comprueba que el mensaje recibido sea del tipo correcto.
                        if(IndexSubindex==obj.statusword)                   % Comprobación de que el mensaje recibido es del objeto del driver requerido 6041h 0h
                            consigna=obj.Data4x8to32(ValueRead);            % Tansformación de la matriz en un número
                            dato=consigna;
                            lecturaOk=true;
                        end
                    end 
                end 
            catch 
                lecturaOk=false;
            end
                
            if lecturaOk==false
                disp("No se pudo leer el estado del driver:"+obj.id);
            else
                obj.statusWord=dec2bin(dato);                    % Se guarda el valor de l estado en formato binario
            end

            dato=obj.statusWord;                        % Se devuelve el dato de la palabra de estados guardado en el atributo del objeto
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % printStatusWord(): este método realiza una llamada a la función
        % getStatusWord() de la clase para refrescar la información del
        % driver conectado por comunicaciones y posteriormente evalúa la
        % información recibida para mostrarla por pantalla. De esta forma
        % el usuario conocerá el estado en el que se encuentra el driver
        % consultado.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function printStatusWord(obj)
                obj.getStatusWord();

                % Escribir por pantalla el estado actual del driver
                mask= dec2bin(hex2dec("6f"),8);                             % Escritura de una máscara que será aplicada a la consigna tratada anteriormente
                Dato=bitand(bin2dec(obj.statusWord),bin2dec(mask));                 % Comparacion de bit con la mascara para filtrar el dato y montar la matriz correcta

                    switch Dato              % Elección del mensaje correcto
                       case 0
                            disp("Driver "+obj.id+" Not Ready to switch on")
                        case 0x60
                            disp("Driver "+obj.id+" Switch on disabled")
                        case 0x40
                            disp("Driver "+obj.id+" Switch on disabled")
                        case 0x21
                            disp("Driver "+obj.id+" Ready to switch on")
                        case 0x23
                            disp("Driver "+obj.id+" Switched on")
                        case 0x27
                            disp("Driver "+obj.id+" Operation enabled")
                        case 0x07
                            disp("Driver "+obj.id+" Quick stop active")
                        case 0x0f
                            disp("Driver "+obj.id+" Fault reaction active")
                        case 0x2f
                            disp("Driver "+obj.id+" Fault reaction active")
                        case 0x08
                            disp("Driver "+obj.id+" Fault")
                        case 0x28
                            disp("Driver "+obj.id+" Fault")
                        otherwise
                            disp("Driver "+obj.id+" Unknown")
                    end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Escalado y desescalado de consignas
       
        % -----------------------------------------------------------------------------------------------------------------------
        % ScallingFactorVelocity(obj): devuelve el valor de escalado de la
        % consigna de velocidad en función de los datos de configuración
        % del driver.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function ScallingFactorVelocity=ScallingFactorVelocity(obj)
            %ESCALA EL FACTOR DE Velocity
            ScallingFactorVelocity = (60.0/(2.0*pi))*(obj.encoderResolucion/60.0)*obj.mlRatio*obj.SampleSL;
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % ScallingFactorPosition(obj): devuelve el valor de escalado de la 
        % consigna de posición en función de los datos de configuración 
        % del driver.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function ScallingFactorPosition = ScallingFactorPosition(obj)
            %ESCALA EL FACTOR DE POSICION
            ScallingFactorPosition = (obj.encoderResolucion*obj.mlRatio)/(2*pi);  
        end
        
        % -----------------------------------------------------------------------------------------------------------------------
        % DescallingFactorPosition(obj): devuelve el valor de la consigna
        % de posición tras ser desescalado. Es decir, se toma el dato
        % recibido por comunicaciones y se transforma en un valor manejable
        % por el usuario.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function dato=DescallingFactorPosition(obj,consigna)
            ScallingFactorPosition = (obj.encoderResolucion*obj.mlRatio)/(2*pi);
            dato = consigna/ScallingFactorPosition;    
        end
    
        % -----------------------------------------------------------------------------------------------------------------------
        % Data32to4x8(obj,consigna): este método transforma el valor de una
        % consigna de 32 bits en una matriz de 4 bytes. Esta matriz será la
        % que se envíe en los mensajes del canal de comunicaciones.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function dato=Data32to4x8(obj,consigna)
            binVal = dec2bin(consigna,32);                                  % Transformación del dato de consigna entero a formato binario
            mask= dec2bin(hex2dec("ff"),32);                                % Escritura de una máscara que será aplicada a la consigna tratada anteriormente
            dato = zeros(1,4);
            for j=0:3
                sz = length(mask);
                Dato=0;
        
                Dato=bitand(bin2dec(binVal),bin2dec(mask));                 % Comporacion de bit con la mascara para filtrar el dato y montar la matriz correcta
                Dato=bitshift(Dato,-(j*8));                                 % Con este algoritmo se busca desplazar el valor de la word comprobada a la posición de los bits menos significativos. Al comprobar en tramas de 8 bits se debe desplazar la información en múltiplos de estos en función de las pasadas dadas                        
                mask=dec2bin(bitshift(bin2dec(mask),8),32);                 % Por medio de esta función desplazamos el valor de la máscara por las distintas posiciones de la consigna para así escrbirila sobre una array de datos
                dato(j+1)=Dato;                                             % Para poder escribir en un array se debe realizar a partir de la posición 1 de esta.
            end    
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % Data4x8to32(obj,consigna): este método transforma las matrices de
        % 4 btes que se reciben por comunicaciones en números de 32 bits,
        % que son interpretables por los usuarios.
        % ----------------------------------------------------------------------------------------------------------------------- 
        function dato=Data4x8to32(obj,consigna)
            dato=0;
            sz=length(consigna);
            for i = 1:sz 
                 dato=dato+bitshift(consigna(i),((i-1)*8));                  % Se va sumando byte a byte el valor recibido por comunicaciones
            end
            
            signo=dec2bin(consigna(sz),8);                                  % Sí el valor recibido es negativo se debe realizar un tratamiento extra
            if(signo(1)=="1")
                dato=bitxor(dato,hex2dec("ffffffff"))*-1;
            end
        end

    % Funciones no implementadas
      
%         % -----------------------------------------------------------------------------------------------------------------------
%         % COMPROBAR SI ESTA EN USO
%         % FALTA POR PROBAR
%         % ----------------------------------------------------------------------------------------------------------------------- 
%         function ScallingFactorAceleration=ScallingFactorAceleration(obj)
%             %ESCALA EL FACTOR DE Aceleration
%             ScallingFactorAceleration = (60.0*obj.encoderResolucion*obj.ratio*obj.sample*obj.sample)/(2*pi);   
%         end

%         % -----------------------------------------------------------------------------------------------------------------------
%         % SE HA DEJADO DE UTILIZAR
%         % ----------------------------------------------------------------------------------------------------------------------- 
%         function profileStart(obj)                                                                  
%             obj.writeSDO (obj.targetPosition, obj.actargetPosition)               % Envío de la posición que debe alcanzar el driver.
%             obj.writePDO1 (obj.runViaControlWord)                                 % Arrancar la acción de alcanzar la posición consignada via ControlWord           
%         end

%         % -----------------------------------------------------------------------------------------------------------------------
%         % COMPROBAR SI ESTA IGUAL QUE EN EL VC Y FUNCIONA
%         % FALTA POR PROBAR
%         % ----------------------------------------------------------------------------------------------------------------------- 
%         function setAceleration(obj,consigna)
%             %AÑADIR UN VALOR DE Aceleration
%             aceleration=consigna*obj.ScallingFactorVelocity();
%             data = obj.Data32to4x8(aceleration);
%             sz = length(data);
%             comando=zeros(1,sz);
%             for i = 1:sz 
%                 str = string(data(i));
%                 comando(i)= str2double(str); 
%             end        
%             obj.writeSDO (obj.targetAceleration, comando)
%         end

    end
end