% -----------------------------------------------------------------------------------------------------------------------
% @author: Cristian Jara Corros
% @NIA: 100440701
% @year: 2023
% @version: v1.0
% @TFGtitle: Librería de comunicación CANopen en MATLAB para un cuello robótico blando
% -----------------------------------------------------------------------------------------------------------------------

% -----------------------------------------------------------------------------------------------------------------------
% Clase para el sensor inercial: migración desde el proyecto en C++
% existente.
% -----------------------------------------------------------------------------------------------------------------------

classdef IMU3DM_GX5_10<handle
    %IMU3DM_GX5_10 Summary of this class goes here
    %   Detailed explanation goes here

    properties (Access = protected)
        % Canal de comunicación del puerto serie del sensor
            puertoSerie;
            frecuencia;
            periodo;

%             registroPitchRoll;
               %Initial offset
    rolloffset;
    pitchoffset; yaw;pitch;roll;
    yawoffset;

    true_yawoff=0;%CR

    tmpYaw;
        %Setting of GyroBias
        estimador;
            bx = -0.002681;
            by = -0.002166;
            bz = -0.001784;
        %Defaults gains used
            Kp = 2.2;
            Ti = 2.65;
            KpQuick = 10;
            TiQuick = 1.25;
        CAL_LOOPS=5; %/number of initial calibration attempts
        % Mensajes disponibles para la comunicación por el puerto serie
        ping = [0x75 0x65 0x01 0x02 0x02 0x01 0xE0 0xC6];
        ok_ping = [0x75 0x65 0x01 0x04 0x04 0xF1 0x01 0x00 0xD5 0x6A];    
        idle = [0x75 0x65 0x01 0x02 0x02 0x02 0xe1 0xc7];
        ok_idle = [0x75 0x65 0x01 0x04 0x04 0xF1 0x02 0x00 0xD6 0x6C];
        imudata1 = [0x75 0x65 0x0c 0x07 0x07 0x08 0x01 0x01 0x05 0x03 0xe8 0xee 0x04];
        imudata100 = [0x75 0x65 0x0c 0x07 0x07 0x08 0x01 0x01 0x05 0x00 0x0a 0x0d 0x20];
        imudata1000 = [0x75 0x65 0x0c 0x07 0x07 0x08 0x01 0x01 0x05 0x00 0x01 0x04 0x17];
        reset = [0x75 0x65 0x01 0x02 0x02 0x7e 0x5d 0x43];
        ok_reset = [0x75 0x65 0x01 0x04 0x04 0xF1 0x7e 0x00 0x52 0x64];
        baudratenew = [0x75 0x65 0x0c 0x07 0x07 0x40 0x01 0x00 0x03 0x84 0x00 0xbc 0x64];
        gyracc1 = [0x75 0x65 0x0c 0x0a 0x0a 0x08 0x01 0x02 0x04 0x03 0xe8 0x05 0x03 0xe8 0xe4 0x0b];
        gyracc50 = [0x75 0x65 0x0c 0x0a 0x0a 0x08 0x01 0x02 0x04 0x00 0x14 0x05 0x00 0x14 0x36 0xd2];
        gyracc100 = [0x75 0x65 0x0c 0x0a 0x0a 0x08 0x01 0x02 0x04 0x00 0x0a 0x05 0x00 0x0a 0x22 0xa0];
        gyracc500 = [0x75 0x65 0x0c 0x0a 0x0a 0x08 0x01 0x02 0x04 0x00 0x02 0x05 0x00 0x02 0x12 0x78];
        gyracc1000 = [0x75 0x65 0x0c 0x0a 0x0a 0x08 0x01 0x02 0x04 0x00 0x01 0x05 0x00 0x01 0x10 0x73];
        setok = [0x75 0x65 0x0c 0x04 0x04 0xF1 0x08 0x00 0xE7 0xBA];
        streamon = [0x75 0x65 0x0c 0x05 0x05 0x11 0x01 0x01 0x01 0x04 0x1a];
        streamoff = [0x75 0x65 0x0c 0x05 0x05 0x11 0x01 0x01 0x00 0x03 0x19];
            ok_streamon = [0x75 0x65 0x0c 0x04 0x04 0xF1 0x11 0x00 0xf0 0xcc];
        polling = [0x75 0x65 0x0c 0x04 0x04 0x01 0x00 0x00 0xef 0xda];
            poll_ready = [0x75 0x65 0x0c 0x04 0x04 0xF1 0x01 0x00 0xe0 0xac];
            poll_data = [0x75 0x65 0x80 0x1c];
    end

    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Constructores y métodos de configuración
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function obj = IMU3DM_GX5_10(comunicationPort,freq)
            obj.estimador=attitude_estimator(true);
            obj.estimador.setMagCalib(0,0,0);
            obj.estimador.setGyroBias(obj.bx,obj.by,obj.bz);

            obj.puertoSerie = serialport(comunicationPort,115200);
            configureTerminator(obj.puertoSerie,"CR")
            obj.setFrequency(freq);            
            obj.setIDLEmode();
            obj.Ping();
            obj.setDevicegetGyroAcc();
            obj.streamon();
            disp("Calibrando")
            obj.calibrate();
            obj.setIDLEmode();
            for i=0:200
                obj.GetPitchRollYaw();
            end
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % setFrequency (obj,freq): nueva frecuencia de muestreo
        % ----------------------------------------------------------------------------------------------------------------------- 
        function setFrequency (obj,freq)
            obj.frecuencia=freq;
            obj.periodo=1/freq;
            %disp('No olvide llamar a la configuración del giroscopio para finalizar la configuración del sensor')
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Métodos de lectura y escritura generales
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function PutMsg(obj,msg)
            try
                %flush(obj.puertoSerie)
                if(length(msg)<=0)
                    % disp("No se han enviado datos")
                end
                for i=1:length(msg)
                    write(obj.puertoSerie,char(str2num(string(msg(i)))),'char');
                end
                flush(obj.puertoSerie);
                pause(0.01);
            catch
                disp("Fallo en PutMsg")
            end
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % ----------------------------------------------------------------------------------------------------------------------- 
        function messagein = getMessage(obj) 
            try
                if obj.puertoSerie.NumBytesAvailable>0
                    % LECTURA DE LOS MENSAJES ENVIADOS
                    data = read(obj.puertoSerie,obj.puertoSerie.NumBytesAvailable,"char");
                    % TRANSFORMAR EL MENSAJE DE CARACTERES A DECIMALES PARA PODER EXTRAER LA INFORMACIÓN RECIBIDA
                    dataDec=zeros(1,strlength(data),"single");
                    for i=1:strlength(data)
                        dataDec(i) = single(data(i));
                    end
                    messagein = dataDec;
                    pause(0.01);
                else 
                    % disp('No hay mensajes para leer');
                    messagein=0;
                end
            catch
            end
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % ----------------------------------------------------------------------------------------------------------------------- 
        function messagein = GetChar(obj) 
            if obj.puertoSerie.NumBytesAvailable>0
                % LECTURA DE LOS MENSAJES ENVIADOS
                data = read(obj.puertoSerie,1,"char");
                % TRANSFORMAR EL MENSAJE DE CARACTERES A DECIMALES PARA PODER EXTRAER LA INFORMACIÓN RECIBIDA
%                 dataDec=zeros(1,strlength(data),"single");
%                 for i=1:strlength(data)
%                     dataDec(i) = single(data(i));
%                 end
                messagein = data;%Dec;
                pause(0.01);
            else 
                % disp('No hay mensajes para leer');
                messagein=0;
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Métodos de petición de datos
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % ----------------------------------------------------------------------------------------------------------------------- 
        function setIDLEmode (obj)
            obj.PutMsg(obj.idle);
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % ----------------------------------------------------------------------------------------------------------------------- 
        function Ping (obj)
            obj.PutMsg(obj.ping);
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % ----------------------------------------------------------------------------------------------------------------------- 
        function setStreamon (obj)
            obj.PutMsg(obj.streamon);
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % ----------------------------------------------------------------------------------------------------------------------- 
        function setStreamoff (obj)
            obj.PutMsg(obj.streamoff);
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % ----------------------------------------------------------------------------------------------------------------------- 
        function Reset (obj)
            obj.PutMsg(obj.reset);
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % ----------------------------------------------------------------------------------------------------------------------- 
        function Polling (obj)
            obj.PutMsg(obj.polling);
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % ----------------------------------------------------------------------------------------------------------------------- 
        function get = getSerialPort (obj)
            get = obj.puertoSerie;
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % ----------------------------------------------------------------------------------------------------------------------- 
        function setDevicegetGyroAcc (obj)
            switch obj.frecuencia
               case 1
                  obj.PutMsg(obj.gyracc1);
                case 50
                  obj.PutMsg(obj.gyracc50);
               case 100
                  obj.PutMsg(obj.gyracc100);
                case 500
                  obj.PutMsg(obj.gyracc500);
                case 1000
                  obj.PutMsg(obj.gyracc1000);
               otherwise
                 %  disp("La frecuencia configurada no es adecuada para la gestión de los datos del sensor");
                  % disp("Por favor, cambie el valor de la frecuencia y vuleva a ejecutar este comando");
                 %  disp("La frecuencia freucencia configurada es: "+obj.frecuencia);
            end
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % ----------------------------------------------------------------------------------------------------------------------- 
        function setDevicegetGyro (obj)
            switch obj.frecuencia
               case 1
                  obj.PutMsg(obj.imudata1);
               case 100
                  obj.PutMsg(obj.imudata100);
                case 1000
                  obj.PutMsg(obj.imudata1000);
               otherwise
                  % disp("La frecuencia configurada no es adecuada para la gestión de los datos del sensor");
                  % disp("Por favor, cambie el valor de la frecuencia y vuleva a ejecutar este comando");
                  % disp("La frecuencia introducida por el usuario es: "+obj.frecuencia);
            end
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % ----------------------------------------------------------------------------------------------------------------------- 
        function dato = GetPitchRollYaw (obj)
            try
                obj.Polling();
                Lectura=obj.getMessage();
                dato=[0 0 0];
                [numRows,numCols] = size(Lectura);
                if(numCols>32 && obj.comprobarMessage(obj.poll_data,Lectura)) 
                    accx = obj.HexToFloat(obj.transforToUint8(Lectura(1,7:10)));
                    accy = obj.HexToFloat(obj.transforToUint8(Lectura(1,11:14)));
                    accz = obj.HexToFloat(obj.transforToUint8(Lectura(1,15:18)));
                    gyrox = obj.HexToFloat(obj.transforToUint8(Lectura(1,21:24)));
                    gyroy = obj.HexToFloat(obj.transforToUint8(Lectura(1,25:28)));
                    gyroz = obj.HexToFloat(obj.transforToUint8(Lectura(1,29:32)));

                    if isnan(accx*accy*accz*gyrox*gyroy*gyroz)
                        obj.estimador.setAttitudeFused(yaw,pitch, roll,1);
                        return;
                    end
                    
                    obj.estimador.update(obj.periodo,gyrox,gyroy,gyroz,accx,accy,accz,0,0,0);

                    DataFused=obj.estimador.fusedData();
                    actualYaw = DataFused(1);%Ahora es FusedData
                    actualPitch = DataFused(2);%Ahora es FusedData
                    actualRoll = DataFused(3);%Ahora es FusedData
                    if abs(gyroz)<0.003
                        gyroz=0;
                    end
                    obj.true_yawoff=obj.true_yawoff+(gyroz*obj.periodo*0.5);
                    actualYaw=obj.true_yawoff;
                        %                     dato = [accx accy accz gyrox gyroy gyroz];
                        %                     disp("La información obtenida de la aceleración es: "+dato(1)+"|"+dato(2)+"|"+dato(3))
                        %                     disp("La información obtenida del giroscopo es: "+dato(4)+"|"+dato(5)+"|"+dato(6))
                        dato=[actualPitch actualRoll actualYaw];    %Datos en radianes
                        
                        % showYaw=rad2deg(actualYaw);            %Datos en grados
                        % showPitch=rad2deg(actualPitch);
                        % showRoll=rad2deg(actualRoll);
                        % dato=[showPitch showRoll showYaw];
                    %disp("Los valores de pitch y roll son: "+showPitch+"|"+showRoll)
                else
                    % disp('Fallo en el mensaje de polling. Faltan de argumentos')
                end
        catch
        end
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % ----------------------------------------------------------------------------------------------------------------------- 
        function calibrate(obj)        
            %We will obtain initial offset to correct it the moment we make measures
            %answer; str; str1; str2; str3; str4; str5;
%             line;
%             c;
%             longitud;
%             descriptor;
            roll=0;
            pitch=0;
            yaw=0; %CR
        
            for h=0:5%OBJ.CAL_LOOPS
               %Reset of the variables to avoid infinite loops.
                answer=""; str=""; str1=""; str2=""; str3=""; str4=""; str5="";
                comp = 0;
                fin = 0;
                c = '';
                longitud = '';
                descriptor = '';
        
                answer = obj.getMessage();

                % Identificación del flag de lectura "UE" a partir de este punto se debe obtener el resto del mensaje, que será el que guarde la posición del giroscopo y demás.
%                 answer = "";
%                 while answer~="ue"
%                     answer=answer+obj.GetChar();
%                     answer
%                     if answer=='u'
%                         answer=answer+obj.GetChar();
%                     else
%                         answer="";
%                     end                
%                 end
%                 descriptor=obj.GetChar();
%                 answer=answer+descriptor;
% 
%                 longitud=obj.GetChar();
%                 answer=answer+longitud;
%                 longitud=hex2dec(obj.transforToUint8(longitud));
%                 
%                 for i=0: longitud
%                     answer=answer+obj.GetChar();
%                 end
                obj.Polling();
                answer=obj.getMessage();
                [numRows,numCols] = size(answer);
                if numCols>32
                    accx = obj.HexToFloat(obj.transforToUint8(answer(1,7:10)));
                    accy = obj.HexToFloat(obj.transforToUint8(answer(1,11:14)));
                    accz = obj.HexToFloat(obj.transforToUint8(answer(1,15:18)));
                    gyrox = obj.HexToFloat(obj.transforToUint8(answer(1,21:24)));
                    gyroy = obj.HexToFloat(obj.transforToUint8(answer(1,25:28)));
                    gyroz = obj.HexToFloat(obj.transforToUint8(answer(1,29:32)));
                    
                    
                    if isnan(accx*accy*accz*gyrox*gyroy*gyroz)
                        return;
                    end
                    obj.estimador.update(obj.periodo,gyrox,gyroy,gyroz,accx*9.81,accy*9.81,accz*9.81,0,0,0);  
                    eulerData=obj.estimador.eulerData();
                    roll = eulerData(3);
                    pitch= eulerData(2);
                    yaw= eulerData(1);
                    dato = [accx accy accz gyrox gyroy gyroz];
%                     disp("La información obtenida de la aceleración es: "+dato(1)+"|"+dato(2)+"|"+dato(3))
%                     disp("La información obtenida del giroscopo es: "+dato(4)+"|"+dato(5)+"|"+dato(6))
                    %First 150 measures are ignored because they are not stable
                    if h>=150 && h<=500
                        obj.rolloffset= obj.rolloffset+ roll ;
                        obj.pitchoffset= obj.pitchoffset +pitch ;
                        obj.yawoffset= obj.yawoffset +yaw;
                    end
                    obj.true_yawoff=0;
                end
               end
            return;
        end

        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % ----------------------------------------------------------------------------------------------------------------------- 
        function dato=Data4x8to32(obj,consigna)                                         %TENEMOS QUE REALIZAR LA RESTRUCTURACIÓN DEL DATO PARA PODER VERLO
            dato=0;
            consigna
            sz=length(consigna);
            for i = 1:sz 
                dato=dato+bitshift(consigna(i),((i-1)*8));
            end
            %Comprobar sí el valor es negativo
            signo=dec2bin(consigna(sz),8);
            if(signo(1)=="1")
                dato=bitxor(dato,hex2dec("ffffffff"))*-1;
            end
            
        end
        function dato=HexToFloat(obj,consigna)
                %typecast(uint32(hex2dec('C0728F5C')),'single')
                a=strcat(dec2hex(consigna(1)),dec2hex(consigna(2)),dec2hex(consigna(3)),dec2hex(consigna(4)));
                dato=typecast(uint32(hex2dec(a)),'single');
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % Los datos de entrada por los mensajes vienen dados como tipo
        % "single" y es necesario transformarlos a numeros enteros para
        % poder trabajar con ellos en las comparaciones y transformaciones
        % a un numero interpretable por el programa
        % ----------------------------------------------------------------------------------------------------------------------- 
        function dato=transforToUint8(obj,consigna)                                         
            dato=[];
            sz=length(consigna);
            for i = 1:sz 
                dato(i)=cast(consigna(i),'uint8');
            end
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % ----------------------------------------------------------------------------------------------------------------------- 
        function comprobacion=comprobarMessage(obj,esperado,recibido)                                        
            sz=length(esperado);
            for i = 1:sz 
                if(str2num(string(esperado(i)))==obj.transforToUint8(recibido(i)))
                    comprobacion=true;
                else
                    comprobacion=false;
                    i=sz+1;
                    % disp('El mensaje de entrada no se corresponde con el esperado.');
                end    
            end
        end
    
    % Funciones no implementadas

    %         % -----------------------------------------------------------------------------------------------------------------------
%         % 
%         % ----------------------------------------------------------------------------------------------------------------------- 
%         function SetRegistroPitchRoll(obj,datos)    
%             try
%                 [numRows,numCols] = size(datos);
%                 if numCols==2
%                     obj.registroPitchRoll=vertcat(obj.registroPitchRoll,datos);
%                 else
%                     disp("Se ha introducido un número incorrecto de datos de entrada")
%                     disp("Unicamente han de ser: [pitch roll]")
%                 end
%             catch
%                 disp("Datos de pitch roll no disponibles para ser registrados")
%             end
%         end
% 
%         % -----------------------------------------------------------------------------------------------------------------------
%         % 
%         % ----------------------------------------------------------------------------------------------------------------------- 
%         function registro=GetRegistroPitchRoll(obj)    
%                 registro=obj.registroPitchRoll;
%         end
% 
%         % -----------------------------------------------------------------------------------------------------------------------
%         % 
%         % ----------------------------------------------------------------------------------------------------------------------- 
%         function ResetRegistroPitchRoll(obj)    
%                 obj.registroPitchRoll=[];
%         end

    end
end

