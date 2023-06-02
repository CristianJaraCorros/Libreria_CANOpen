% -----------------------------------------------------------------------------------------------------------------------
% @author: Cristian Jara Corros
% @NIA: 100440701
% @year: 2023
% @version: v1.0
% @TFGtitle: Librería de comunicación CANopen en MATLAB para un cuello robótico blando
% -----------------------------------------------------------------------------------------------------------------------

% -----------------------------------------------------------------------------------------------------------------------
% getInclinacionOrientacion(pitch,roll): el usuario introducirá los datos
% de pitch y roll para ser transformados en una matriz de dos posiciones en
% las que se guardarán la inclinación y la orientación de la articulación.
% ----------------------------------------------------------------------------------------------------------------------- 
function [inclinacion orientacion]=getInclinacionOrientacion(pitch,roll)    
    try
        inclinacion = sqrt(pitch^2 + roll^2) * (180 / pi);
        orientacion = ((atan2(roll, pitch) * (180 / pi)));

%         % Conditions for having 360 degrees in orientation
%         if orientacion > 0
%             orientacion = orientacion;
%         end
%         if orientacion < 0
%             orientacion = 360 - abs(orientacion);
%         end
    catch
        % disp("Datos de pitch roll no disponibles para ser registrados")
    end

end
