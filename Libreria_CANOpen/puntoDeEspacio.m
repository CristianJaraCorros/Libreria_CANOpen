% -----------------------------------------------------------------------------------------------------------------------
% @author: Cristian Jara Corros
% @NIA: 100440701
% @year: 2023
% @version: v1.0
% @TFGtitle: Librería de comunicación CANopen en MATLAB para un cuello robótico blando
% -----------------------------------------------------------------------------------------------------------------------

% -----------------------------------------------------------------------------------------------------------------------
% puntoDeEspacio(longitud,inclinacion,orientacion): esta función combina
% el dato de la longitud del elemento con la información obtenida de
% la inclinación y orientación de la articulación en un momento concreto.
% Se obtiene como resultado una posición en un espacio tridimensional que
% posteriormente podrá ser usado para realizar una gráfica en 3D para
% visualizar el movimiento de la articulación robótica.
% ----------------------------------------------------------------------------------------------------------------------- 

function [x y z]=puntoDeEspacio(longitud,inclinacion,orientacion)    
    try
        s0=longitud*(1-cos(inclinacion))/inclinacion;
        t0=longitud*sin(inclinacion)/inclinacion;

        x=s0*cos(orientacion);
        y=s0*sin(orientacion);
        z=t0;
    catch
        disp("Datos de pitch roll introducidos no son validos")
    end

end