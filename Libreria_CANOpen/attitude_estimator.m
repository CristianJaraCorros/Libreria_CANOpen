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
classdef attitude_estimator<handle
    %IMU3DM_GX5_10 Summary of this class goes here
    %   Detailed explanation goes here

    properties (Access = protected)
        m_fusedValid=false;m_eulerValid=false;m_KpQuick;m_TiQuick;
        m_Fhat= [0 0 0];m_Kp;m_Ti;m_QLTime;m_lambda;
        m_magCalib = [0 0 0];m_accMethod;
        m_bhat= [0 0 0 0];ME_COUNT=0;
        m_Ehat=[0 0 0];ME_DEFAULT=0;
        m_Qhat=[0 0 0 0];
        m_base=[0 0 0];
        m_FhatHemi;
        m_wold=[0 0 0];
        m_w=[0 0 0];
        m_dQold=[0 0 0 0];
        m_dQ=[0 0 0 0];
        m_Qtilde=[0 0 0 0];
        m_Qy=[0 0 0 0];
        m_omega=[0 0 0];
        m_Ry=[0 0 0 0 0 0 0 0 0];
        
        ME_FUSED_YAW=0;
        
        ACC_TOL_SQ = 1e-12*1e-12;
        QY_NORM_TOL_SQ = 1e-12*1e-12;
        QHAT_NORM_TOL_SQ = 1e-12*1e-12;
        XGYG_NORM_TOL_SQ = 1e-12*1e-12;
        WEZE_NORM_TOL_SQ = 1e-12*1e-12;
        ZGHAT_ABS_TOL = 1e-12;
    end

    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Constructores y métodos de configuración
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function obj = attitude_estimator(quickLearn) %boleano
                obj.resetAll(quickLearn);
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function dato = fusedData(obj) % Returns the fused yaw, pitch,roll of the current attitude estimate (1st of the fused angles).
            if obj.m_fusedValid==false
                obj.updateFused();
            end
            dato=obj.m_Fhat;
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function setMagCalib(obj,mtx,mty,mtz) %Sets the magnetometer calibration vector to use in the update functions. This should be the value of `(magX,magY,magZ)` that corresponds to a true orientation of identity.
            obj.m_magCalib(1)=mtx;  
            obj.m_magCalib(2)=mty;
            obj.m_magCalib(3)=mtz;
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function setGyroBias(obj,bx,by,bz) % Resets the current estimated gyro bias to a particular vector value.
            obj.m_bhat(1)=bx;obj.m_bhat(2)=by;obj.m_bhat(3)=bz;
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function dato = eulerData(obj) % Returns the ZYX Euler yaw,pitch, roll of the current attitude estimate.
            if obj.m_eulerValid==false
                obj.updateEuler();
            end
            dato=obj.m_Ehat;
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function updateFused(obj)
	        % These calculations rely on the assumption that m_Qhat is a unit quaternion!
	        %
	        % The output ranges are:
	        %   Fused yaw:    psi  = m_Fhat[0]  is in (-pi,pi]
	        %   Fused pitch: theta = m_Fhat[1]  is in [-pi/2,pi/2]
	        %   Fused roll:   phi  = m_Fhat[2]  is in [-pi/2,pi/2]
	        %   Hemisphere:    h   = m_FhatHemi is in {-1,1} (stored as {false,true} respectively)
        
	        % Calculate and wrap the fused yaw
	        obj.m_Fhat(1) = 2.0*atan2(obj.m_Qhat(4),obj.m_Qhat(1)); % Output of atan2 is [-pi,pi], so this expression is in [-2*pi,2*pi]
	        if obj.m_Fhat(1) > pi 
                obj.m_Fhat(1)= obj.m_Fhat(1) - 2*pi;    % Fhat[0] is now in [-2*pi,pi]
            end
	        if obj.m_Fhat(1) <= -pi 
                obj.m_Fhat(1)= obj.m_Fhat(1) + 2*pi;   % Fhat[0] is now in (-pi,pi]
            end
        
	        % Calculate the fused pitch and roll
	        stheta = 2.0*(obj.m_Qhat(3)*obj.m_Qhat(1) - obj.m_Qhat(2)*obj.m_Qhat(4));
	        sphi   = 2.0*(obj.m_Qhat(3)*obj.m_Qhat(4) + obj.m_Qhat(2)*obj.m_Qhat(1)); 
            if abs(stheta)>1    % Coerce stheta to [-1,1]
                stheta=abs(stheta)/stheta;
            end 
            if abs(sphi)>1    % Coerce sphi to [-1,1]
                sphi=abs(sphi)/sphi;
            end
	        obj.m_Fhat(2) = asin(stheta);
	        obj.m_Fhat(3) = asin(sphi);
        
            if (isnan(obj.m_Fhat(1)*obj.m_Fhat(2)*obj.m_Fhat(3))) 
                disp("ERROR m_Fhat");
            end
        
	        % Calculate the hemisphere of the rotation
	        obj.m_FhatHemi = (0.5 - (obj.m_Qhat(2)*obj.m_Qhat(2) + obj.m_Qhat(3)*obj.m_Qhat(3)) >= 0.0);
        
	        % Set the fused angles valid flag
	        obj.m_fusedValid = true;
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function updateEuler(obj)
	        % These calculations rely on the assumption that m_Qhat is a unit quaternion!
	        %
	        % The output ranges are:
	        %   Yaw:    psi  = m_Ehat[0] is in (-pi,pi]
	        %   Pitch: theta = m_Ehat[1] is in [-pi/2,pi/2]
	        %   Roll:   phi  = m_Ehat[2] is in (-pi,pi]
        
	        % Calculate pitch
	        stheta = 2.0*(obj.m_Qhat(1)*obj.m_Qhat(3) - obj.m_Qhat(4)*obj.m_Qhat(2));
	        if abs(stheta)>1    % Coerce stheta to [-1,1]
                stheta=abs(stheta)/stheta;
            end 
	        obj.m_Ehat(2) = asin(stheta);
        
            % Calculate yaw and roll
	        ysq = obj.m_Qhat(3)*obj.m_Qhat(3);
	        obj.m_Ehat(1) = atan2(obj.m_Qhat(1)*obj.m_Qhat(4)+obj.m_Qhat(2)*obj.m_Qhat(3), 0.5-(ysq+obj.m_Qhat(4)*obj.m_Qhat(4)));
	        obj.m_Ehat(3) = atan2(obj.m_Qhat(1)*obj.m_Qhat(2)+obj.m_Qhat(3)*obj.m_Qhat(4), 0.5-(ysq+obj.m_Qhat(2)*obj.m_Qhat(2)));
        
            if isnan(obj.m_Ehat(1)*obj.m_Ehat(2)*obj.m_Ehat(3)) 
                disp("ERROR m_Ehat");
            end
	        % Set the Euler angles valid flag
            obj.m_eulerValid = true;
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function setAttitude(obj,w,x,y,z)
            % Calculate the quaternion square norm
	        qscale = w*w + x*x + y*y + z*z;
        
	        % Update the current attitude estimate
	        if(qscale < obj.QHAT_NORM_TOL_SQ) % Reset the attitude estimate to the identity orientation if the norm is too close to zero
		        obj.m_Qhat = [1 0 0 0];	
            else
		        qscale = 1.0 / sqrt(qscale);
		        obj.m_Qhat = [(qscale*w) (qscale*x) (qscale*y) (qscale*z)];
	        end
        
	        % Reset the alternative representation validity flags
	        obj.m_eulerValid = false;
	        obj.m_fusedValid = false;
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function setAttitudeFused(obj,yaw,pitch,roll,hemi)
            % Precalculate the sin values
            sth  = sin(pitch);
            sphi = sin(roll);
            % Calculate the sine sum criterion
            crit = sth*sth + sphi*sphi;
            % Calculate the tilt angle alpha
            if crit >= 1
	            alpha = 2*pi;
            elseif hemi==true
	            alpha = acos(sqrt(1-crit));
            else
                alpha = acos(-sqrt(1-crit));
            end
            % Calculate the tilt axis gamma
            gamma = atan2(sth,sphi);
            % Evaluate the required intermediate angles
            halpha  = 0.5*alpha;
            hpsi    = 0.5*yaw;
            hgampsi = gamma + hpsi;
            % Precalculate trigonometric terms involved in the quaternion expression
            chalpha = cos(halpha);
            shalpha = sin(halpha);
            chpsi = cos(hpsi);
            shpsi = sin(hpsi);
            chgampsi = cos(hgampsi);
            shgampsi = sin(hgampsi);
            % Calculate the required quaternion orientation and set it as the current attitude estimate
            obj.setAttitude(chalpha*chpsi, shalpha*chgampsi, shalpha*shgampsi, chalpha*shpsi);
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function update(obj, dt, gyroX, gyroY, gyroZ, accX, accY, accZ, magX, magY, magZ)
	        % The algorithm implemented in this function is based on the Passive Complementary Filter described in:
	        % --> R. Mahoney, T. Hamel, and J.-M. Pflimlin, "Nonlinear complementary filters on the special orthogonal group",
	        % --> IEEE Transactions on Automatic Control, vol. 53, no. 5, pp. 1203-1218.
	        % Do not modify *anything* unless you know *exactly* what you're doing (even if you think you're not changing anything significant),
	        % or it could affect the unconditional and unadulterated robustness of the entire estimator. This applies to updateQy() as well.
        
	        % Update lambda
	        if(obj.m_lambda < 1.0)
		        obj.m_lambda=obj.m_lambda + dt / obj.m_QLTime;
		        if(obj.m_lambda <= 0.0) obj.m_lambda = 0.0; end
		        if(obj.m_lambda >= 1.0) obj.m_lambda = 1.0; end
            end
	        % Calculate the required filter gains for this update (Note: Ki = Kp / Ti)
	        Kp = obj.m_lambda*obj.m_Kp + (1-obj.m_lambda)*obj.m_KpQuick;
	        Ti = obj.m_lambda*obj.m_Ti + (1-obj.m_lambda)*obj.m_TiQuick;
        
	        % Save the old values of the required variables
	        obj.m_wold(1) = obj.m_w(1);
	        obj.m_wold(2) = obj.m_w(2);
	        obj.m_wold(3) = obj.m_w(3);
	        obj.m_dQold(1) = obj.m_dQ(1);
	        obj.m_dQold(2) = obj.m_dQ(2);
	        obj.m_dQold(3) = obj.m_dQ(3);
	        obj.m_dQold(4) = obj.m_dQ(4);
        
	        % Calculate Qy, the current quaternion orientation measurement, based on the acc and mag readings
	        obj.updateQy(accX, accY, accZ, magX, magY, magZ); % Writes to m_Qy internally, and never fails...
        
	        % Calculate the rotational error between the current Qhat and the new measured Qy
	        obj.m_Qtilde(1) = obj.m_Qhat(1)*obj.m_Qy(1) + obj.m_Qhat(2)*obj.m_Qy(2) + obj.m_Qhat(3)*obj.m_Qy(3) + obj.m_Qhat(4)*obj.m_Qy(4);
	        obj.m_Qtilde(2) = obj.m_Qhat(1)*obj.m_Qy(2) - obj.m_Qhat(2)*obj.m_Qy(1) - obj.m_Qhat(3)*obj.m_Qy(4) + obj.m_Qhat(4)*obj.m_Qy(3);
	        obj.m_Qtilde(3) = obj.m_Qhat(1)*obj.m_Qy(3) + obj.m_Qhat(2)*obj.m_Qy(4) - obj.m_Qhat(3)*obj.m_Qy(1) - obj.m_Qhat(4)*obj.m_Qy(2);
	        obj.m_Qtilde(4) = obj.m_Qhat(1)*obj.m_Qy(4) - obj.m_Qhat(2)*obj.m_Qy(3) + obj.m_Qhat(3)*obj.m_Qy(2) - obj.m_Qhat(4)*obj.m_Qy(1);
        
	        % Calculate the angular velocity feedback term required to act in the direction of reducing Qtilde
	        wscale = 2 * Kp * obj.m_Qtilde(1);
	        obj.m_w(1) = wscale * obj.m_Qtilde(2);
	        obj.m_w(2) = wscale * obj.m_Qtilde(3);
	        obj.m_w(3) = wscale * obj.m_Qtilde(4);
        
	        % Update the estimated gyro bias (trapezoidal integration of -Ki*w)
	        bscale = 0.5 * dt / Ti;
	        obj.m_bhat(1)= obj.m_bhat(1) - bscale*(obj.m_w(1) + obj.m_wold(1));
	        obj.m_bhat(2)= obj.m_bhat(2) - bscale*(obj.m_w(2) + obj.m_wold(2));
	        obj.m_bhat(3)= obj.m_bhat(3) - bscale*(obj.m_w(3) + obj.m_wold(3));
        
	        % Calculate the required (combined predictive/corrective) angular velocity to apply to our current attitude estimate
	        obj.m_omega(1) = gyroX - obj.m_bhat(1) + obj.m_w(1);
	        obj.m_omega(2) = gyroY - obj.m_bhat(2) + obj.m_w(2);
	        obj.m_omega(3) = gyroZ - obj.m_bhat(3) + obj.m_w(3);
        
	        % Convert the calculated angular velocity into a quaternion velocity (the missing factor of 0.5 here has been taken into dscale below)
	        obj.m_dQ(1) = -obj.m_Qhat(2)*obj.m_omega(1) - obj.m_Qhat(3)*obj.m_omega(2) - obj.m_Qhat(4)*obj.m_omega(3);
	        obj.m_dQ(2) =  obj.m_Qhat(1)*obj.m_omega(1) + obj.m_Qhat(3)*obj.m_omega(3) - obj.m_Qhat(4)*obj.m_omega(2);
	        obj.m_dQ(3) =  obj.m_Qhat(1)*obj.m_omega(2) - obj.m_Qhat(2)*obj.m_omega(3) + obj.m_Qhat(4)*obj.m_omega(1);
	        obj.m_dQ(4) =  obj.m_Qhat(1)*obj.m_omega(3) + obj.m_Qhat(2)*obj.m_omega(2) - obj.m_Qhat(3)*obj.m_omega(1);
        
	        % Update the attitude estimate using the calculated quaternion velocity (trapezoidal integration of dQ)
	        dscale = 0.25 * dt; % The extra factor of 0.5 here comes from the omission thereof in the calculation above
	        obj.m_Qhat(1)= obj.m_Qhat(1) + dscale*(obj.m_dQ(1) + obj.m_dQold(1));
	        obj.m_Qhat(2)= obj.m_Qhat(2) + dscale*(obj.m_dQ(2) + obj.m_dQold(2));
	        obj.m_Qhat(3)= obj.m_Qhat(3) + dscale*(obj.m_dQ(3) + obj.m_dQold(3));
	        obj.m_Qhat(4)= obj.m_Qhat(4) + dscale*(obj.m_dQ(4) + obj.m_dQold(4));
        
	        % Renormalise the current attitude estimate
	        qscale = obj.m_Qhat(1)*obj.m_Qhat(1) + obj.m_Qhat(2)*obj.m_Qhat(2) + obj.m_Qhat(3)*obj.m_Qhat(3) + obj.m_Qhat(4)*obj.m_Qhat(4);
	        if(qscale < obj.QHAT_NORM_TOL_SQ)
                obj.reset(true); 
                return; % The quaternion is so far away from being normalised (almost zero norm when it should be 1) that something must be dreadfully wrong... (avoid potential division by zero below)
            end
            qscale = 1.0 / sqrt(qscale);
	        obj.m_Qhat(1)=obj.m_Qhat(1)*qscale;
	        obj.m_Qhat(2)=obj.m_Qhat(2)*qscale;
	        obj.m_Qhat(3)=obj.m_Qhat(3)*qscale;
	        obj.m_Qhat(4)=obj.m_Qhat(4)*qscale;
        
	        % Reset the alternative representation validity flags
	        obj.m_eulerValid = false;
	        obj.m_fusedValid = false;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% BORRAR

% disp("m_fhat: "+obj.m_Fhat(1)+" "+obj.m_Fhat(2)+" "+obj.m_Fhat(3))
% disp("m_magCalib: "+obj.m_magCalib(1)+" "+obj.m_magCalib(2)+" "+obj.m_magCalib(3))
% disp("m_bhat: "+obj.m_bhat(1)+" "+obj.m_bhat(2)+" "+obj.m_bhat(3)+" "+obj.m_bhat(4))
% disp("m_Ehat: "+obj.m_Ehat(1)+" "+obj.m_Ehat(2)+" "+obj.m_Ehat(3))
% disp("m_Qhat: "+obj.m_Qhat(1)+" "+obj.m_Qhat(2)+" "+obj.m_Qhat(3)+" "+obj.m_Qhat(4))
% disp("m_base: "+obj.m_base(1)+" "+obj.m_base(2)+" "+obj.m_base(3))
% disp("m_wold: "+obj.m_wold(1)+" "+obj.m_wold(2)+" "+obj.m_wold(3))
% disp("m_w: "+obj.m_w(1)+" "+obj.m_w(2)+" "+obj.m_w(3))
% disp("m_dQold: "+obj.m_dQold(1)+" "+obj.m_dQold(2)+" "+obj.m_dQold(3)+" "+obj.m_dQold(4))
% disp("m_dQ: "+obj.m_dQ(1)+" "+obj.m_dQ(2)+" "+obj.m_dQ(3)+" "+obj.m_dQ(4))
% disp("m_Qtilde: "+obj.m_Qtilde(1)+" "+obj.m_Qtilde(2)+" "+obj.m_Qtilde(3)+" "+obj.m_Qtilde(4))
% disp("m_omega: "+obj.m_omega(1)+" "+obj.m_omega(2)+" "+obj.m_omega(3))
% disp("m_Ry: "+obj.m_Ry(1)+" "+obj.m_Ry(2)+" "+obj.m_Ry(3)+" "+obj.m_Ry(4)+" "+obj.m_Ry(5)+" "+obj.m_Ry(6)+" "+obj.m_Ry(7)+" "+obj.m_Ry(8)+" "+obj.m_Ry(9))

        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function updateQy(obj,accX,accY,accZ,magX,magY,magZ)
	        % This function generates a quaternion Qy (saved in the class member variable m_Qy) representing
	        % the measured orientation of the body (i.e. the body-fixed coordinate system) relative to the
	        % global coordinate system. It is referred to as the 'measured' orientation of the body, as the
	        % quaternion derives from the given acc and mag measurements. The acc vector is guaranteed to
	        % be respected, no matter what the inputs to this function and the current state estimate are.
	        % That is to say, the generated Qy is guaranteed to be such that if the robot is exactly at the
	        % rotation Qy, and gravity points down the negative global z-axis, then the given value of acc
	        % is analytically expected to be measured in terms of body-fixed coordinates. The mag vector is
	        % respected as long as it can be used to resolve the acc vector into a full 3D orientation, given
	        % the current magnetometer calibration vector.
	        % 
	        % This function uses the value of m_magCalib = (mtx,mty,mtz). Only the projection of this vector
	        % onto the global xG-yG plane is considered however, which is equivalent to (mtx,mty,0). As such,
	        % only the values of mtx and mty are used by this function, and m_magCalib is effectively taken
	        % to be (mtx,mty,0), irrespective of what the value of mtz is. The only difference this makes is
	        % in the comments, where for example ||m_magCalib|| is used as shorthand for sqrt(mtx^2 + mty^2).
	        % In actual fact, all that really matters about the m_magCalib vector is its heading, because in
	        % the calculation of Qy that's all it's effectively reduced to. More precisely stated, only the
	        % value of atan2(mty,mtx) is important. The magnetometer readings do not affect any component of
	        % the resulting rotation other than the yaw about the measured acceleration vector!
	        %
	        % The output may be Qy or -Qy. That is, Qy isn't forced to live in any particular half of
	        % the quaternion space, and any code that uses the outputs of this function must be able to deal
	        % with that. This is usually not a problem, as both quaternion multiplication and quaternion-
	        % vector rotation implicitly handle this ambiguity.
	        %
	        % The rotation matrix m_Ry (if used) is stored as a 1D array of 9 doubles, arranged as follows:
	        %        G    | Ry[0] Ry[1] Ry[2] |   | xBx yBx zBx |   | xGx xGy xGz |   B
	        %   Ry =  R = | Ry[3] Ry[4] Ry[5] | = | xBy yBy zBy | = | yGx yGy yGz | =  R'
	        %        B    | Ry[6] Ry[7] Ry[8] |   | xBz yBz zBz |   | zGx zGy zGz |   G
	        %
	        % Thus the standard orthonormal basis vectors of the global coordinate system written in the
	        % body-fixed coordinate system are (G => Global coords, B => Body-fixed coords):
	        %   xGhat = G(1,0,0) = B(xGx, xGy, xGz) = B(Ry[0], Ry[1], Ry[2])
	        %   yGhat = G(0,1,0) = B(yGx, yGy, yGz) = B(Ry[3], Ry[4], Ry[5])
	        %   zGhat = G(0,0,1) = B(zGx, zGy, zGz) = B(Ry[6], Ry[7], Ry[8])
	        % Most of the algorithm cases in this function work by constructing the xGhat, yGhat and zGhat
	        % vectors (in body-fixed coordinates). These are then used to populate the rotation matrix Ry,
	        % which is then converted into the required quaternion Qy.
	        %
	        % We define the coordinate system H to be the coordinate system obtained when rotating the body-
	        % fixed coordinate system B by the inverse of m_Qhat. In other words, H is our current estimate
	        % of how the global coordinate system is oriented, without having considered the sensor
	        % measurements of this cycle yet. The coordinate system G is defined as our 'measured' orientation
	        % of the global csys, based on the sensor measurements of this cycle. The csys G *always* has zGhat
	        % as its z-axis (i.e. respects the acc measurement). The orientations of the xG and yG axes are
	        % then calculated either using the mag measurement, or m_Qhat and some assumption about the yaw.
	        % Letting Qy represent the quaternion rotation from G to B (our 'measured' orientation of B for
	        % this cycle, towards which m_Qhat limits via the filter equations) we have the following; taking
	        % B to be our reference csys, G is the csys obtained by the inverse rotation of Qy, and H is the
	        % csys obtained by the inverse rotation of m_Qhat.
	        
	        % Declare variables
	        
	        % Calculate the norm (squared) of the acc measurement
	        naccsq = accX*accX + accY*accY + accZ*accZ; % = ||acc||^2	        
	        % If the acc square norm is too close to zero then we actually have no idea where we are, so just set Qy to the identity quaternion and return
	        if naccsq < obj.ACC_TOL_SQ
                obj.m_Qy= [1 0 0 0]; % Qy -> (w x y z)
		        return;
            end	  

	        % Define zGhat as the unit vector pointing in the direction of acc (the opposite direction to which gravity is pointing)
	        % Note: To machine precision the resulting zGhat will have unit norm, i.e. ||zGhat|| == 1
	        naccinv = 1.0 / sqrt(naccsq);
	        obj.m_Ry(7) = accX * naccinv; % zGhat -> x = zGx
	        obj.m_Ry(8) = accY * naccinv; % zGhat -> y = zGy
	        obj.m_Ry(9) = accZ * naccinv; % zGhat -> z = zGz
	        
	        % Project mag into the xG-yG plane (in body-fixed coordinates) using the formula mtilde := mag - dot(mag,zGhat)*zGhat
	        % Note: To machine precision the resulting mtilde is perpendicular to zGhat, and ||mtilde|| = ||mag||*sin(angle(acc,mag)),
	        %       where the angle is taken to be in the range [0,pi).
	        dot = magX*obj.m_Ry(7) + magY*obj.m_Ry(8) + magZ*obj.m_Ry(9); % dot(mag,zGhat)
	        magX= magX - dot*obj.m_Ry(7);                              % mtilde -> x
	        magY= magY - dot*obj.m_Ry(8);                              % mtilde -> y
	        magZ= magZ - dot*obj.m_Ry(9);                              % mtilde -> z
	        
	        % Generate a second orthogonal basis vector for the xG-yG plane, complementary to mtilde, using the cross product (defines the csys X = m_base / Y = mtilde / Z = zGhat)
	        % Note: To machine precision ||m_base|| == ||mtilde|| and dot(m_base,mtilde) == 0
	        obj.m_base(1) = magY*obj.m_Ry(9) - magZ*obj.m_Ry(8); % m_base -> x
	        obj.m_base(2) = magZ*obj.m_Ry(7) - magX*obj.m_Ry(9); % m_base -> y
	        obj.m_base(3) = magX*obj.m_Ry(8) - magY*obj.m_Ry(7); % m_base -> z
	        
	        % Calculate orthogonal xG and yG such that mtilde is collinear with m_magCalib (a vector in xG-yG-zG coordinates) projected onto the xG-yG plane
	        % Note: To machine precision ||xG|| == ||yG|| == ||m_magCalib||*||mtilde|| == ||m_magCalib||*||mag||*sin(angle(acc,mag)), where the
	        %       angle is taken to be in the range [0,pi). Zero xG/yG arise iff m_magCalib is zero, or acc and mag are collinear.
	        obj.m_Ry(1) = obj.m_magCalib(2)*obj.m_base(1) + obj.m_magCalib(1)*magX; % xG -> x
	        obj.m_Ry(2) = obj.m_magCalib(2)*obj.m_base(2) + obj.m_magCalib(1)*magY; % xG -> y
	        obj.m_Ry(3) = obj.m_magCalib(2)*obj.m_base(3) + obj.m_magCalib(1)*magZ; % xG -> z
	        obj.m_Ry(4) = obj.m_magCalib(2)*magX - obj.m_magCalib(1)*obj.m_base(1); % yG -> x
	        obj.m_Ry(5) = obj.m_magCalib(2)*magY - obj.m_magCalib(1)*obj.m_base(2); % yG -> y
	        obj.m_Ry(6) = obj.m_magCalib(2)*magZ - obj.m_magCalib(1)*obj.m_base(3); % yG -> z
	        
	        % Calculate the xG and yG vector (square) norms
	        % Note: The calculated nxG and nyG should theoretically be identical.
	        nxG = obj.m_Ry(1)*obj.m_Ry(1) + obj.m_Ry(2)*obj.m_Ry(2) + obj.m_Ry(3)*obj.m_Ry(3); % = ||xG||^2
	        nyG = obj.m_Ry(4)*obj.m_Ry(4) + obj.m_Ry(5)*obj.m_Ry(5) + obj.m_Ry(6)*obj.m_Ry(6); % = ||yG||^2
	        
	        % Check whether the basis vector generation was successful (non-zero xG/yG)
	        % Note: We check both nxG and nyG, even though to machine precision we should have nxG == nyG.
	        if((nxG < obj.XGYG_NORM_TOL_SQ) || (nyG < obj.XGYG_NORM_TOL_SQ))
		        % This IF block executes if the algorithm was unable to resolve the acc and mag measurements into a consistent and unambiguous
		        % 3D rotation. Assuming the m_magCalib is not the zero vector, this occurs iff acc and mag are collinear. In this case the mag
		        % vector gives us no valid information and we are forced to discard it. We must now generate a full 3D rotation from just the
		        % acc vector (i.e. from zGhat), using some additional assumption in order to be able to resolve the global yaw. This additional
		        % assumption should ideally make the 'measured' orientation match as closely as possible to the current orientation estimate,
		        % while respecting the zGhat vector.
        
		        % Use the required acc-only resolution method to generate a full 3D orientation measurement Qy from zGhat (=> m_Ry[6-8)) and m_Qhat
		        if obj.m_accMethod == obj.ME_FUSED_YAW
			        % Fused Yaw Method
        
			        % We attempt to use the current orientation estimate m_Qhat to resolve the zGhat vector into a complete 3D rotation that agrees
			        % "as much as possible" in a relative sense with m_Qhat. The code in this IF block assumes that m_Qhat is a unit quaternion.
			        % Within reason this code can deal with numeric deviations thereof, but eventually these act to reduce the numerical accuracy of
			        % the computed values. The term "fused yaw" refers to the yaw (about the z-axis) as defined by the first component of the fused
			        % angles representation.
        
			        % If the resolution of zGhat using m_Qhat is successful, then a quaternion is generated directly and returned. If on the other
			        % hand the resolution attempt fails, a fallback solution is used, and this IF block outputs orthogonal xG (stored in m_Ry[0-2])
			        % and yG (stored in m_Ry[3-5]), and the respective square norms of each, nxG = ||xG||^2 and nyG = ||yG||^2, such that xG-yG-zGhat
			        % defines the 'measured' global coordinate system relative to the body-fixed coordinate system (the inverse rotation of m_Qy).
        
			        % Rotate zGhat = BzG (in B coordinates) by m_Qhat = HqB to get the same vector HzG, but expressed in H coordinates
			        % Note: HzG = m_Qhat * BzG * conj(m_Qhat), and assuming m_Qhat is a unit quaternion we have ||HzG|| = ||BzG|| as expected.
			        perpx = 2*(obj.m_Qhat(3)*obj.m_Ry(9) - obj.m_Qhat(4)*obj.m_Ry(8)); % perp -> x
			        perpy = 2*(obj.m_Qhat(4)*obj.m_Ry(7) - obj.m_Qhat(2)*obj.m_Ry(9)); % perp -> y
			        perpz = 2*(obj.m_Qhat(2)*obj.m_Ry(8) - obj.m_Qhat(3)*obj.m_Ry(7)); % perp -> z
			        HzGx = obj.m_Ry(7) + obj.m_Qhat(1)*perpx - obj.m_Qhat(4)*perpy + obj.m_Qhat(3)*perpz; % HzG -> x
			        HzGy = obj.m_Ry(8) + obj.m_Qhat(4)*perpx + obj.m_Qhat(1)*perpy - obj.m_Qhat(2)*perpz; % HzG -> y
			        HzGz = obj.m_Ry(9) - obj.m_Qhat(3)*perpx + obj.m_Qhat(2)*perpy + obj.m_Qhat(1)*perpz; % HzG -> z        
			        % Calculate the Qytilde, the unnormalised quaternion rotation from G to B, such that G and H have no fused yaw relative to one another
			        % Note: We have that the required (unnormalised) GqHtilde = (1+HzGz, HzGy, -HzGx, 0), which can easily be seen to have no fused yaw.
			        %       We then have Qytilde = GqBtilde = GqHtilde * HqB = (1+HzGz, HzGy, -HzGx, 0) * m_Qhat.
			        HzGztilde = 1.0 + HzGz;
			        obj.m_Qy(1) = obj.m_Qhat(1)*HzGztilde - obj.m_Qhat(2)*HzGy + obj.m_Qhat(3)*HzGx; % Qytilde -> w
			        obj.m_Qy(2) = obj.m_Qhat(2)*HzGztilde + obj.m_Qhat(1)*HzGy - obj.m_Qhat(4)*HzGx; % Qytilde -> x
			        obj.m_Qy(3) = obj.m_Qhat(3)*HzGztilde - obj.m_Qhat(4)*HzGy - obj.m_Qhat(1)*HzGx; % Qytilde -> y
			        obj.m_Qy(4) = obj.m_Qhat(4)*HzGztilde + obj.m_Qhat(3)*HzGy + obj.m_Qhat(2)*HzGx; % Qytilde -> z        
			        % Calculate the square norm of the generated quaternion Qy
			        % Note: This should only ever be zero if m_Qhat is zero, or if HzG = (0,0,-1).
			        nQysq = obj.m_Qy(1)*obj.m_Qy(1) + obj.m_Qy(2)*obj.m_Qy(2) + obj.m_Qy(3)*obj.m_Qy(3) + obj.m_Qy(4)*obj.m_Qy(4); % = |Qy|^2        
			        % Normalise and return the generated quaternion, if possible, or use a fallback solution to calculate an alternative 'measured' acc-only quaternion Qy
			        if nQysq >= obj.QY_NORM_TOL_SQ
				        % Normalise Qy and return
				        nQyinv = 1.0 / sqrt(nQysq); % = 1/|Qy|
				        obj.m_Qy(1)= obj.m_Qy(1)* nQyinv; % Qy -> w
				        obj.m_Qy(2)= obj.m_Qy(2)* nQyinv; % Qy -> x
				        obj.m_Qy(3)= obj.m_Qy(3)* nQyinv; % Qy -> y
				        obj.m_Qy(4)= obj.m_Qy(4)* nQyinv; % Qy -> z
				        return;
                    else
				        % This case only executes if the z-axes of G and H are exactly opposite, hence bringing the fused yaw representation of the relative
				        % rotation between G and H to a singularity point. Said in another way, the above generation of m_Qy only fails if m_Qhat is zero
				        % (assumed not to ever be the case), acc is zero (in which case the above code wouldn't have executed anyway because that case is
				        % checked earlier), and/or HzG = (0,0,-1) (implying that zGhat and zH point in exactly opposite directions). So in essence, this case
				        % only executes if the acc vector points *exactly* in the wrong direction according to the current orientation estimate m_Qhat.
        
				        % Calculate orthogonal xG and yG (orthogonal to each other and zGhat) such that the ZYX yaw of H with respect to G is zero (as opposed to the fused yaw now)
				        % Note: The critical underlying observation behind this efficient calculation is that the ZYX yaw of a rotation rotates the global
				        %       x-axis so that it is collinear with the [global] projection of the body-fixed (i.e. rotated) x-axis into the [global] xy-plane.
				        %       As such, the problem becomes finding an xG such that xG projected into the xG-yG plane (== xG) is equal to xh (the global x-axis
				        %       expressed in body-fixed coordinates if the body is at an orientation of m_Qhat) projected into the xG-yG plane. Hence we never
				        %       actually need to calculate a ZYX yaw, and can instead just set xG to be exactly the projection of xh into the plane perpendicular
				        %       to zGhat. If m_Qhat is a unit quaternion, to machine precision ||0.5*xh|| = 0.5 and ||xG|| = ||yG|| = 0.5*sin(angle(xh,zGhat)),
				        %       where the angle is taken to be in the range [0,pi]. Zero xG/yG arise iff xh and zGhat/acc are collinear. yG is calculated as the
				        %       cross product of zGhat and xG.
				        obj.m_Ry(1) = 0.5 - (obj.m_Qhat(3)*obj.m_Qhat(3) + obj.m_Qhat(4)*obj.m_Qhat(4)); % 0.5*xh -> x
				        obj.m_Ry(2) = obj.m_Qhat(2)*obj.m_Qhat(3) - obj.m_Qhat(4)*obj.m_Qhat(1);         % 0.5*xh -> y
				        obj.m_Ry(3) = obj.m_Qhat(2)*obj.m_Qhat(4) + obj.m_Qhat(3)*obj.m_Qhat(1);         % 0.5*xh -> z
				        dot = obj.m_Ry(1)*obj.m_Ry(7) + obj.m_Ry(2)*obj.m_Ry(8) + obj.m_Ry(3)*obj.m_Ry(9);   % = dot(0.5*xh,zGhat)
				        obj.m_Ry(1)= obj.m_Ry(1) - dot*obj.m_Ry(7);                                      % xG -> x
				        obj.m_Ry(2)= obj.m_Ry(2) - dot*obj.m_Ry(8);                                      % xG -> y
				        obj.m_Ry(3)= obj.m_Ry(3) - dot*obj.m_Ry(9);                                      % xG -> z
				        obj.m_Ry(4)= obj.m_Ry(4) - obj.m_Ry(3)*obj.m_Ry(8) - obj.m_Ry(2)*obj.m_Ry(9);                 % yG -> x
				        obj.m_Ry(5)= obj.m_Ry(5) - obj.m_Ry(1)*obj.m_Ry(9) - obj.m_Ry(3)*obj.m_Ry(7);                 % yG -> y
				        obj.m_Ry(6)= obj.m_Ry(6) - obj.m_Ry(2)*obj.m_Ry(7) - obj.m_Ry(1)*obj.m_Ry(8);                 % yG -> z
        
				        % Calculate the xG and yG vector (square) norms
				        % Note: The calculated nxG and nyG should theoretically be identical.
				        nxG = obj.m_Ry(1)*obj.m_Ry(1) + obj.m_Ry(2)*obj.m_Ry(2) + obj.m_Ry(3)*obj.m_Ry(3); % = ||xG||^2
				        nyG = obj.m_Ry(4)*obj.m_Ry(4) + obj.m_Ry(5)*obj.m_Ry(5) + obj.m_Ry(6)*obj.m_Ry(6); % = ||yG||^2
        
				        % Check whether the basis vector generation was successful (non-zero xG/yG)
				        % Note: We check both nxG and nyG, even though to machine precision we should have nxG == nyG.
				        if((nxG < obj.XGYG_NORM_TOL_SQ) || (nyG < obj.XGYG_NORM_TOL_SQ))
					        % The fallback ZYX yaw based method only fails if m_Qhat is such that xh is collinear with acc. That is to say, zGhat is collinear
					        % with xH. In this case it is impossible however for zGhat to also be antiparallel to zH, and so in all such cases we should have
					        % that the initial fused yaw method solution worked. As such, this case should be impossible to reach. We handle it anyway though
					        % so that there is no conceivable way that a division by zero can occur.
        
					        % Set Qy to the identity quaternion and return
					        obj.m_Qy = [1 0 0 0]; % Qy -> (w x y z)
					        return;
                        end
                    end
		        elseif(m_accMethod == obj.ME_ABS_FUSED_YAW)
			        % Absolute Fused Yaw Method
        
			        % We attempt to use the current orientation estimate m_Qhat to resolve the zGhat vector into a complete 3D rotation that agrees
			        % "as much as possible" in an absolute sense with m_Qhat. The code in this IF block assumes that m_Qhat is a unit quaternion.
			        % Within reason this code can deal with numeric deviations thereof, but eventually these act to reduce the numerical accuracy of
			        % the computed values. The term "fused yaw" refers to the yaw (about the z-axis) as defined by the first component of the fused
			        % angles representation.
        
			        % If the resolution of zGhat using m_Qhat is successful, then a quaternion is generated directly and returned. If on the other
			        % hand the resolution attempt fails, a fallback solution is used, and this IF block outputs orthogonal xG (stored in m_Ry[0-2])
			        % and yG (stored in m_Ry[3-5]), and the respective square norms of each, nxG = ||xG||^2 and nyG = ||yG||^2, such that xG-yG-zGhat
			        % defines the 'measured' global coordinate system relative to the body-fixed coordinate system (the inverse rotation of m_Qy).
        
			        % Declare variables
			        wEhat; zEhat;
        
			        % Calculate the quaternion that corresponds to the pure fused yaw component of m_Qhat
			        % Note: If we let m_Qhat = (wE,xE,yE,zE), then the fused yaw component quaternion is just (wE,0,0,zE) normalised to unit magnitude.
			        %       The resulting normalised quaternion is then (wEhat,0,0,zEhat). It is important to stress that this shortcut for extracting
			        %       the yaw is in fact true and proven for fused yaw, but is *not* true for ZYX yaw, as is sometimes erroneously thought. Care
			        %       needs to be taken in the vicinity of wE = zE = 0, to avoid divisions by zero.
			        zEsq = obj.m_Qhat(4)*obj.m_Qhat(4);           % zE^2
			        nwEzEsq = obj.m_Qhat(1)*obj.m_Qhat(1) + zEsq; % wE^2 + zE^2 = ||(we,0,0,zE)||^2
			        if(nwEzEsq >= obj.WEZE_NORM_TOL_SQ)
				        nwEzEinv = 1.0 / sqrt(nwEzEsq);
				        wEhat = obj.m_Qhat(1) * nwEzEinv;
				        zEhat = obj.m_Qhat(4) * nwEzEinv;
			        else % <-- Too close to wE = zE = 0 to just apply normalisation => We are in the vicinity of fused yaw instability of m_Qhat, and must resort to using the (in that region) stable ZYX yaw instead
				        hZYXyawE = 0.5*atan2(obj.m_Qhat(1)*obj.m_Qhat(4)+obj.m_Qhat(2)*obj.m_Qhat(3), 0.5-(obj.m_Qhat(3)*obj.m_Qhat(3)+zEsq)); % Half of the ZYX yaw of m_Qhat
				        wEhat = cos(hZYXyawE);
				        zEhat = sin(hZYXyawE);
                    end
			        % Calculate a quaternion that when normalised corresponds to a Qy that has the same fused yaw as m_Qhat (more precisely, the same fused yaw as (wEhat,0,0,zEhat))
			        % Note: Taking zGhat = (zGx,zGy,zGz), the quaternion that respects this zGhat and has zero fused yaw is given by the normalisation
			        %       of (1+zGz,zGy,-zGx,0). Thus, the quaternion that respects the given zGhat, and has fused yaw equal to that of the pure yaw
			        %       quaternion (wEhat,0,0,zEhat), is given by the normalisation of the quaternion product (wEhat,0,0,zEhat)*(1+zGz,zGy,-zGx,0).
			        %       This expands to Qytilde = (wEhat*(1+zGz), zGx*zEhat+zGy*wEhat, zGy*zEhat-zGx*wEhat, zEhat*(1+zGz)).
			        zGztilde = 1.0 + obj.m_Ry(9);         % 1 + zGz
			        obj.m_Qy(1) = wEhat*zGztilde;                % Qytilde -> w (unnormalised)
			        obj.m_Qy(2) = wEhat*obj.m_Ry(8) + zEhat*obj.m_Ry(7); % Qytilde -> x (unnormalised)
			        obj.m_Qy(3) = zEhat*obj.m_Ry(8) - wEhat*obj.m_Ry(7); % Qytilde -> y (unnormalised)
			        obj.m_Qy(4) = zEhat*zGztilde;                % Qytilde -> z (unnormalised)
        
			        % Calculate the square norm of the generated quaternion
			        % Note: We should have nQysq = ||Qytilde||^2 = 1 + 2*zGz + zGx^2 + zGy^2 + zGz^2 = 2*(1+zGz) as zGhat is a unit vector.
			        nQysq = obj.m_Qy(1)*obj.m_Qy(1) + obj.m_Qy(2)*obj.m_Qy(2) + obj.m_Qy(3)*obj.m_Qy(3) + obj.m_Qy(4)*obj.m_Qy(4);
        
			        % Normalise and return the generated quaternion, if possible, or use a fallback solution to calculate an alternative 'measured' acc-only quaternion Qy
			        if(nQysq >= obj.QY_NORM_TOL_SQ)
				        % Normalise m_Qy and return
				        nQyinv = 1.0 / sqrt(nQysq);
				        obj.m_Qy(1)= obj.m_Qy(1) * nQyinv; % Qy -> w
				        obj.m_Qy(2)= obj.m_Qy(2) * nQyinv; % Qy -> x
				        obj.m_Qy(3)= obj.m_Qy(3) * nQyinv; % Qy -> y
				        obj.m_Qy(4)= obj.m_Qy(4) * nQyinv; % Qy -> z
				        return;
                    else
				        % This case executes if the m_Qy that was generated using the method above was essentially zero, and thus couldn't be normalised
				        % to obtain a unit 'measured' orientation as required. The square norm of Qytilde is expected to be 2*(1+zGz), and thus this case
				        % invokes when zGz is very close to -1, that is, when zGhat is very close to (0,0,-1). We identify this situation as being exactly
				        % when m_Qy is close to the fused yaw singularity. As the fused yaw goes unstable near this singularity, we calculate a fallback
				        % solution by matching ZYX yaws (instead of matching fused yaws like before). That is, we calculate a rotation matrix m_Ry that
				        % respects zGhat and has the same ZYX yaw as the quaternion (wEhat,0,0,zEhat) (but not in general the same ZYX yaw as m_Qhat!).
				        % We define xh to be the projection of xBhat = B(1,0,0) onto the global xG-yG plane. This is the plane perpendicular to the known
				        % vector zGhat. The angle from xG to xh about the zG axis is equivalent to the ZYX yaw of the body-fixed frame (i.e. of m_Qy).
        
				        % Calculate xh in both the global and body-fixed coordinate systems
				        % Note: We have that xh = G(cos(psiE),sin(psiE),0), where psiE is the (independent of definition) yaw of (wEhat,0,0,zEhat).
				        %       We also have xh = xBhat - dot(xBhat,zGhat)*zGhat = B(1-zGx^2,-zGx*zGy,-zGx*zGz) and yh = cross(zGhat,xBhat) = B(0,zGz,-zGy).
				        %       The magnitudes of xh and yh should to machine precision be ||xh||^2 = ||yh||^2 = zGy^2 + zGz^2, assuming that zGhat is a
				        %       unit vector. The vectors xh, yh and zGhat together form an orthogonal basis. The double angle formulas for sin and cos are
				        %       used to evaluate cos(psiE) and sin(psiE), using that we know that wEhat = cos(psiE/2) and zEhat = sin(psiE/2). Note that
				        %       yh isn't actually explicitly calculated here, it is used implicitly in the next step.
				        cpsiE = wEhat*wEhat - zEhat*zEhat; % cos(psiE) = cos(psiE/2)^2 - sin(psiE/2)^2
				        spsiE = 2.0*wEhat*zEhat;           % sin(psiE) = 2*sin(psiE/2)*cos(psiE/2)
				        xhx = 1.0 - obj.m_Ry(7)*obj.m_Ry(7);       % xh -> x = 1 - zGx^2
				        xhy = -obj.m_Ry(7)*obj.m_Ry(8);            % xh -> y = -zGx*zGy
				        xhz = -obj.m_Ry(7)*obj.m_Ry(9);            % xh -> z = -zGx*zGz
        
				        % Calculate orthogonal xG and yG such that the global and body-fixed representations of xh match (i.e. xh is xG yawed by psiE)
				        % Note: The required equations are xG = xh*cos(psiE) - yh*sin(psiE) and yG = cross(zGhat,xG), with neither xG nor yG necessarily
				        %       coming out a unit vector. Important however is that ||xh||^2 = ||yh||^2, implying that ||xG||^2 = A. As ||yG||^2 = ||xG||^2
				        %       this means that to machine precision we have ||xG||^2 = ||yG||^2 = ||xh||^2 = ||yh||^2 = zGy^2 + zGz^2.
				        obj.m_Ry(1) = xhx*cpsiE;                         % xG -> x
				        obj.m_Ry(2) = xhy*cpsiE - obj.m_Ry(9)*spsiE;         % xG -> y
				        obj.m_Ry(3) = xhz*cpsiE + obj.m_Ry(8)*spsiE;         % xG -> z
				        obj.m_Ry(4) = obj.m_Ry(8)*obj.m_Ry(3) - obj.m_Ry(2)*obj.m_Ry(9); % yG -> x
				        obj.m_Ry(5) = obj.m_Ry(9)*obj.m_Ry(1) - obj.m_Ry(3)*obj.m_Ry(7); % yG -> y
				        obj.m_Ry(6) = obj.m_Ry(7)*obj.m_Ry(2) - obj.m_Ry(1)*obj.m_Ry(8); % yG -> z
        
				        % Calculate the xG and yG vector (square) norms
				        % Note: The calculated nxG and nyG should theoretically be identical.
				        nxG = obj.m_Ry(1)*obj.m_Ry(1) + obj.m_Ry(2)*obj.m_Ry(2) + obj.m_Ry(3)*obj.m_Ry(3); % = ||xG||^2
				        nyG = obj.m_Ry(4)*obj.m_Ry(4) + obj.m_Ry(5)*obj.m_Ry(5) + obj.m_Ry(6)*obj.m_Ry(6); % = ||yG||^2
        
				        % Check whether the basis vector generation was successful (non-zero xG/yG)
				        % Note: We check both nxG and nyG, even though to machine precision we should have nxG == nyG.
				        if((nxG < obj.XGYG_NORM_TOL_SQ) || (nyG < obj.XGYG_NORM_TOL_SQ))
					        % This case should be physically impossible, as for the matching of fused yaws to have failed we must have zGz very close to -1,
					        % and for the matching of ZYX yaws to have failed we must have zGy^2 + zGz^2 very close to 0. It is clearly not possible for
					        % both of these conditions to be true at the same time. To cover all bases though, and avoid absolutely all remotely conceivable
					        % possibilities of division by zero, we handle this dud case separately.
        
					        % Set Qy to the identity quaternion and return
					        obj.m_Qy = [1 0 0 0]; % Qy -> (w x y z)
					        return;
                        end
                    end
		        elseif(m_accMethod == obj.ME_ZYX_YAW)
			        % ZYX Yaw Method
			        
			        % We attempt to use the current orientation estimate m_Qhat to resolve the zGhat vector into a complete 3D rotation that agrees
			        % "as much as possible" with m_Qhat. The code in this IF block assumes that m_Qhat is a unit quaternion. Within reason this code
			        % can deal with numeric deviations thereof, but eventually these act to reduce the numerical accuracy of the computed values.
			        % The terms "ZYX yaw" and "ZXY yaw" in the comments below refer to the yaw of a rotation (about the z-axis) in the case of ZYX
			        % and ZXY Euler angles respectively.
			        %
			        % The output of this IF block in all cases is orthogonal xG (stored in m_Ry[0-2]) and yG (stored in m_Ry[3-5]), and the respective
			        % square norms of each, nxG = ||xG||^2 and nyG = ||yG||^2, such that xG-yG-zGhat defines the 'measured' global coordinate system
			        % relative to the body-fixed coordinate system (the inverse rotation of m_Qy).
			        
			        % Calculate orthogonal xG and yG (orthogonal to each other and zGhat) such that the ZYX yaw of H with respect to G is zero
			        % Note: The critical underlying observation behind this efficient calculation is that the ZYX yaw of a rotation rotates the global
			        %       x-axis so that it is collinear with the [global] projection of the body-fixed (i.e. rotated) x-axis into the [global] xy-plane.
			        %       As such, the problem becomes finding an xG such that xG projected into the xG-yG plane (== xG) is equal to xh (the global x-axis
			        %       expressed in body-fixed coordinates if the body is at an orientation of m_Qhat) projected into the xG-yG plane. Hence we never
			        %       actually need to calculate a ZYX yaw, and can instead just set xG to be exactly the projection of xh into the plane perpendicular
			        %       to zGhat. If m_Qhat is a unit quaternion, to machine precision ||0.5*xh|| = 0.5 and ||xG|| = ||yG|| = 0.5*sin(angle(xh,zGhat)),
			        %       where the angle is taken to be in the range [0,pi]. Zero xG/yG arise iff xh and zGhat/acc are collinear. yG is calculated as the
			        %       cross product of zGhat and xG.
			        obj.m_Ry(1) = 0.5 - (obj.m_Qhat(3)*obj.m_Qhat(3) + obj.m_Qhat(4)*obj.m_Qhat(4)); % 0.5*xh -> x
			        obj.m_Ry(2) = obj.m_Qhat(2)*obj.m_Qhat(3) - obj.m_Qhat(4)*obj.m_Qhat(1);         % 0.5*xh -> y
			        obj.m_Ry(3) = obj.m_Qhat(2)*obj.m_Qhat(4) + obj.m_Qhat(3)*obj.m_Qhat(1);         % 0.5*xh -> z
			        dot = obj.m_Ry(1)*obj.m_Ry(7) + obj.m_Ry(2)*obj.m_Ry(8) + obj.m_Ry(3)*obj.m_Ry(9);   % = dot(0.5*xh,zGhat)
			        obj.m_Ry(1)= obj.m_Ry(1) - dot*obj.m_Ry(7);                                      % xG -> x
			        obj.m_Ry(2)= obj.m_Ry(2) - dot*obj.m_Ry(8);                                      % xG -> y
			        obj.m_Ry(3)= obj.m_Ry(3) - dot*obj.m_Ry(9);                                      % xG -> z
			        obj.m_Ry(4) = obj.m_Ry(3)*obj.m_Ry(8) - obj.m_Ry(2)*obj.m_Ry(9);                 % yG -> x
			        obj.m_Ry(5) = obj.m_Ry(1)*obj.m_Ry(9) - obj.m_Ry(3)*obj.m_Ry(7);                 % yG -> y
			        obj.m_Ry(6) = obj.m_Ry(2)*obj.m_Ry(7) - obj.m_Ry(1)*obj.m_Ry(8);                 % yG -> z
        
			        % Calculate the xG and yG vector (square) norms
			        % Note: The calculated nxG and nyG should theoretically be identical.
			        nxG = obj.m_Ry(1)*obj.m_Ry(1) + obj.m_Ry(2)*obj.m_Ry(2) + obj.m_Ry(3)*obj.m_Ry(3); % = ||xG||^2
			        nyG = obj.m_Ry(4)*obj.m_Ry(4) + obj.m_Ry(5)*obj.m_Ry(5) + obj.m_Ry(6)*obj.m_Ry(6); % = ||yG||^2
        
			        % Check whether the basis vector generation was successful (non-zero xG/yG)
			        % Note: We check both nxG and nyG, even though to machine precision we should have nxG == nyG.
			        if((nxG < obj.XGYG_NORM_TOL_SQ) || (nyG < obj.XGYG_NORM_TOL_SQ))
				        % This IF block executes if the mag vector had to be discarded (i.e. if m_magCalib is zero, or acc and mag are collinear) and
				        % m_Qhat is such that xh is collinear with acc. That is, mag was discarded and our current attitude estimate places the global
				        % x-axis in the same (or 180 degree opposite) direction as the acc we just measured. Assuming m_Qhat is a unit quaternion, this
				        % can only happen if our estimate is *exactly* out by 90 degrees, and perchance in *exactly* the right direction. If m_Qhat
				        % deviates from unit norm by more than a negligible eps-ish amount however (which it should never do), then funkier and slightly
				        % counterintuitive things can happen that produce further special cases below. The xh (ZYX) and yh (ZXY) methods produce
				        % identical results for collinear zGhat and zh. As the angle between these two vectors increases, the xh and yh methods
				        % continuously (but only gradually) start to differ more and more in their output (in terms of global yaw only though, the zGhat
				        % vector always points directly opposite to the measured acc). As the angle between zGhat (opposite of measured acc) and zh
				        % (current estimate of the up direction in body-fixed coordinates) approaches 90 degrees, there are two singularities
				        % (corresponding to gimbal lock of the two different Euler angle conventions) that make the output of the xh and yh methods
				        % start to differ vastly and in an unstable way. For example, close to zGhat == xh the xh output becomes extremely sensitive to
				        % small variations in zGhat (and nxG/nyG tend to zero, which is the condition that is checked for by this IF block), while the
				        % yh output remains stable, and vice versa for zGhat == yh. We wish to have a continuous xG/yG output for as much of the domain
				        % as possible though, so we nominally always use xh (as it corresponds to matching global yaw in a sense consistent with our
				        % nominal ZYX Euler angle convention) unless it produces an effectively zero output, in which case we switch to yh, which in
				        % this neighbourhood is guaranteed to be stable (with the unit quaternion technical caveat). The switching is of course a
				        % locally discontinuous operation though, but we really have no choice here due to the nasty directional dependence of the
				        % singularities. So in summary, the important points to take home from all this are:
				        % 1) This yh method/IF block will practically never execute, but it cannot be neglected.
				        % 2) The singularities and regions of progressive instability in xG/yG do NOT depend on the absolute orientation of acc and
				        %    m_Qhat at all (i.e. it doesn't matter if we're upright, upside-down or sideways), it depends only on the *relative*
				        %    orientation/rotation between acc and m_Qhat. What's more, it can only even possibly be a problem if acc and m_Qhat
				        %    disagree in what direction is up by almost exactly 90 degrees, and by coincidence *exactly* in the wrong direction.
        
				        % Calculate orthogonal xG and yG so that the ZXY yaw of m_Qy, the rotation defined by the resulting orthogonal basis xG-yG-zGhat, is equal to the ZXY yaw of m_Qhat
				        % Note: The critical underlying observation behind this efficient calculation is that the ZXY yaw of a rotation rotates the global
				        %       y-axis so that it is collinear with the [global] projection of the body-fixed (i.e. rotated) y-axis into the [global] xy-plane.
				        %       As such, the problem becomes finding a yG such that yG projected into the xG-yG plane (== yG) is equal to yh (the global y-axis
				        %       expressed in body-fixed coordinates if the body is at an orientation of m_Qhat) projected into the xG-yG plane. Hence we never
				        %       actually need to calculate a ZXY yaw, and can instead just set yG to be exactly the projection of yh into the plane perpendicular
				        %       to zGhat. If m_Qhat is a unit quaternion, to machine precision ||0.5*yh|| = 0.5 and ||yG|| = ||xG|| = 0.5*sin(angle(yh,zGhat)),
				        %       where the angle is taken to be in the range [0,pi]. Zero xG/yG arise iff yh and zGhat/acc are collinear. xG is calculated as the
				        %       cross product of yG and zGhat.
				        obj.m_Ry(4) = obj.m_Qhat(2)*obj.m_Qhat(3) + obj.m_Qhat(4)*obj.m_Qhat(1);         % 0.5*yh -> x
				        obj.m_Ry(5) = 0.5 - (obj.m_Qhat(2)*obj.m_Qhat(2) + obj.m_Qhat(4)*obj.m_Qhat(4)); % 0.5*yh -> y
				        obj.m_Ry(6) = obj.m_Qhat(3)*obj.m_Qhat(4) - obj.m_Qhat(2)*obj.m_Qhat(1);         % 0.5*yh -> z
				        dot = obj.m_Ry(4)*obj.m_Ry(7) + obj.m_Ry(5)*obj.m_Ry(8) + obj.m_Ry(6)*obj.m_Ry(9);   % = dot(0.5*yh,zGhat)
				        obj.m_Ry(4)= obj.m_Ry(4) - dot*obj.m_Ry(7);                                      % yG -> x
				        obj.m_Ry(5)= obj.m_Ry(5) - dot*obj.m_Ry(8);                                      % yG -> y
				        obj.m_Ry(6)= obj.m_Ry(6) - dot*obj.m_Ry(9);                                      % yG -> z
				        obj.m_Ry(1) = obj.m_Ry(5)*obj.m_Ry(9) - obj.m_Ry(6)*obj.m_Ry(8);                 % xG -> x
				        obj.m_Ry(2) = obj.m_Ry(6)*obj.m_Ry(7) - obj.m_Ry(4)*obj.m_Ry(9);                 % xG -> y
				        obj.m_Ry(3) = obj.m_Ry(4)*obj.m_Ry(8) - obj.m_Ry(5)*obj.m_Ry(7);                 % xG -> z
        
				        % Calculate the xG and yG vector (square) norms
				        % Note: The calculated nxG and nyG should theoretically be identical.
				        nxG = obj.m_Ry(1)*obj.m_Ry(1) + obj.m_Ry(2)*obj.m_Ry(2) + obj.m_Ry(3)*obj.m_Ry(3); % = ||xG||^2
				        nyG = obj.m_Ry(4)*obj.m_Ry(4) + obj.m_Ry(5)*obj.m_Ry(5) + obj.m_Ry(6)*obj.m_Ry(6); % = ||yG||^2
        
				        % Check whether the basis vector generation was successful (non-zero xG/yG)
				        % Note: We check both nxG and nyG, even though to machine precision we should have nxG == nyG.
				        if((nxG < obj.XGYG_NORM_TOL_SQ) || (nyG < obj.XGYG_NORM_TOL_SQ))
					        % Ok, so you're asking me why I'm even checking this case, seeing as it's impossible anyway, right? Right...?
					        % Well, almost. Somewhat surprisingly it *is* actually possible for the calculated xh and yh to turn out collinear, even though
					        % the corresponding expressions used to calculate them are orthogonal - as long as m_Qhat is a unit quaternion! That exactly is
					        % the catch. This entire function was written with the mindset that it should never ever break, even if it receives completely
					        % rubbish inputs. In that case it should simply output whatever it thinks is most appropriate, and *never* start performing
					        % divisions by zero or such. No matter what happens, this function *must* be guaranteed to return a valid non-Inf/NaN quaternion,
					        % or the effects of one bad call could ripple through and break the entire attitude estimator, which is not robust. I've managed
					        % to prove mathematically that the previously calculated xh and yh are collinear iff m_Qhat is a pure vector quaternion of norm
					        % 1/sqrt(2). If, furthermore, the resulting collinear xh and yh also happen to be collinear with the measured acc, AND with the
					        % measured mag (or if m_magCalib is zero), then this case is invoked! If this is the case, then we know that m_Qhat is useless,
					        % and so we are forced to discard it also. In order to still be able to resolve our single acc measurement into a full 3D
					        % orientation, some extra (arbitrary) assumption is required. The two assumptions in use below amount to assumptions of zero yaw
					        % in terms of either ZYX yaw or ZXY yaw, with preference to the (nominal) former convention for Euler angles.
        
					        % Check whether zGhat is collinear with (1,0,0), and produce an appropriate output either way, with the assumption of zero ZYX yaw or ZXY yaw
					        % Note: If zGhat is collinear with (1,0,0), then the code in the first case doesn't work because (1,0,0) is the ZYX gimbal lock situation.
					        %       In this case however we know that zGhat can't also be collinear with (0,1,0), the ZXY gimbal lock situation, so it is safe to
					        %       calculate a new xG/yG pair based on a zero ZXY yaw assumption without any further checks.
					        if((abs(obj.m_Ry(8)) >= obj.ZGHAT_ABS_TOL) || (abs(obj.m_Ry(9)) >= obj.ZGHAT_ABS_TOL)) % If zGhat is *not* collinear with (1,0,0)...
						        % Assume zero ZYX yaw: xG is the projection of (1,0,0) into the plane perpendicular to zGhat, yG is the cross product of zGhat and xG
						        % Note: To machine precision ||xG|| = ||yG|| = sin(angle(zGhat,(1,0,0))), where the angle is taken to be in the range (0,pi).
						        obj.m_Ry(1) = 1.0 - obj.m_Ry(7)*obj.m_Ry(7); % xG -> x
						        obj.m_Ry(2) = -obj.m_Ry(7)*obj.m_Ry(8);      % xG -> y
						        obj.m_Ry(3) = -obj.m_Ry(7)*obj.m_Ry(9);      % xG -> z
						        obj.m_Ry(4) = 0.0;                   % yG -> x
						        obj.m_Ry(5) = obj.m_Ry(9);               % yG -> y
						        obj.m_Ry(6) = -obj.m_Ry(8);              % yG -> z
					        else % If zGhat *is* collinear with (1,0,0), and hence not collinear with (0,1,0) (as it is a unit vector)...
						        % Assume zero ZXY yaw: yG is the projection of (0,1,0) into the plane perpendicular to zGhat, xG is the cross product of yG and zGhat
						        % Note: To machine precision ||xG|| = ||yG|| = sin(angle(zGhat,(0,1,0))), where the angle is taken to be in the range [0,pi].
						        %       This case is only invoked if m_Qhat is either exactly (0,+-1/sqrt(2),0,0) or (0,0,0,+-1/sqrt(2)), acc is non-zero and
						        %       along the body-fixed x-axis, and mag is collinear with acc or m_magCalib is zero.
						        obj.m_Ry(1) = obj.m_Ry(9);               % xG -> x
						        obj.m_Ry(2) = 0.0;                   % xG -> y
						        obj.m_Ry(3) = -obj.m_Ry(7);              % xG -> z
						        obj.m_Ry(4) = -obj.m_Ry(8)*obj.m_Ry(7);      % yG -> x
						        obj.m_Ry(5) = 1.0 - obj.m_Ry(8)*obj.m_Ry(8); % yG -> y
						        obj.m_Ry(6) = -obj.m_Ry(8)*obj.m_Ry(9);      % yG -> z
                            end
        
					        % Calculate the xG and yG vector (square) norms
					        % Note: The calculated nxG and nyG should theoretically be identical.
					        nxG = obj.m_Ry(1)*obj.m_Ry(1) + obj.m_Ry(2)*obj.m_Ry(2) + obj.m_Ry(3)*obj.m_Ry(3); % = ||xG||^2
					        nyG = obj.m_Ry(4)*obj.m_Ry(4) + obj.m_Ry(5)*obj.m_Ry(5) + obj.m_Ry(6)*obj.m_Ry(6); % = ||yG||^2
        
					        % Check whether the basis vector generation was successful (non-zero xG/yG)
					        % Note: We check both nxG and nyG, even though to machine precision we should have nxG == nyG.
					        if((nxG < obj.XGYG_NORM_TOL_SQ) || (nyG < obj.XGYG_NORM_TOL_SQ))
						        % This case should be physically impossible, i.e. both theoretically and numerically. To cover all bases though,
						        % and avoid absolutely all remotely conceivable possibilities of division by zero, we handle this dud case separately.
        
						        % Set Qy to the identity quaternion and return
						        obj.m_Qy = [1 0 0 0]; % Qy -> (w x y z)
						        return;
                            end
                        end
                    end
		        else % <-- Unrecognised acc-only resolution method, should never happen!
			        % Set Qy to the identity quaternion and return
			        obj.m_Qy = [1 0 0 0]; % Qy -> (w x y z)
			        return;
                end
            end
        
	        % Normalise xG and yG to obtain an orthonormal basis (in conjunction with zGhat) that forms the rows of the orthogonal rotation matrix m_Ry
	        % Note: The calculated orthonormal basis <xGhat, yGhat, zGhat> is placed in the rows of this matrix (as opposed to the columns) as we want
	        %       the inverse (i.e. transpose) rotation for m_Ry. That is, the global to body-fixed frame rotation, not the body-fixed to global rotation.
	        nxG = 1.0 / sqrt(nxG);
	        nyG = 1.0 / sqrt(nyG);
	        obj.m_Ry(1)= obj.m_Ry(1) * nxG; % xGhat -> x = xGx
	        obj.m_Ry(2)= obj.m_Ry(2) * nxG; % xGhat -> y = xGy
	        obj.m_Ry(3)= obj.m_Ry(3) * nxG; % xGhat -> z = xGz
	        obj.m_Ry(4)= obj.m_Ry(4) * nyG; % yGhat -> x = yGx
	        obj.m_Ry(5)= obj.m_Ry(5) * nyG; % yGhat -> y = yGy
	        obj.m_Ry(6)= obj.m_Ry(6) * nyG; % yGhat -> z = yGz
        
	        % Declare variables
	        r; s; t;
        
	        % Convert the rotation matrix m_Ry into the quaternion m_Qy
	        % Note: m_Qy and -m_Qy are both valid and completely equivalent outputs here, so we have
	        %       the freedom to arbitrarily choose the sign of *one* of the quaternion parameters.
	        t = obj.m_Ry(1) + obj.m_Ry(5) + obj.m_Ry(9);
	        if(t >= 0.0)                                          % Option 1: Centred at identity rotation... [Condition ensures |w| >= 0.5, WLOG choose the sign w >= 0.5)
		        r = sqrt(1.0 + t);                                % = 2*|w|
		        s = 0.5/r;                                        % = 1/(4*|w|)
		        obj.m_Qy(1) = 0.5*r;                                  % = |w|           = w*sgn(w) = w
		        obj.m_Qy(2) = s*(obj.m_Ry(8) - obj.m_Ry(6));                  % = (4xw)/(4*|w|) = x*sgn(w) = x
		        obj.m_Qy(3) = s*(obj.m_Ry(3) - obj.m_Ry(7));                  % = (4yw)/(4*|w|) = y*sgn(w) = y
		        obj.m_Qy(4) = s*(obj.m_Ry(4) - obj.m_Ry(2));                  % = (4zw)/(4*|w|) = z*sgn(w) = z
	        elseif((obj.m_Ry(9) >= obj.m_Ry(5)) && (obj.m_Ry(9) >= obj.m_Ry(1))) % Option 2: Centred at 180 deg z-rotation... [Conditions ensure |z| > 0.5, WLOG choose the sign z > 0.5)
		        r = sqrt(1.0 - (obj.m_Ry(1) + obj.m_Ry(5) - obj.m_Ry(9)));    % = 2*|z|
		        s = 0.5/r;                                        % = 1/(4*|z|)
		        obj.m_Qy(1) = s*(obj.m_Ry(4) - obj.m_Ry(2));                  % = (4zw)/(4*|z|) = w*sgn(z) = w
		        obj.m_Qy(2) = s*(obj.m_Ry(3) + obj.m_Ry(7));                  % = (4xz)/(4*|z|) = x*sgn(z) = x
		        obj.m_Qy(3) = s*(obj.m_Ry(8) + obj.m_Ry(6));                  % = (4yz)/(4*|z|) = y*sgn(z) = y
		        obj.m_Qy(4) = 0.5*r;                                  % = |z|           = z*sgn(z) = z
	        elseif(obj.m_Ry(5) >= obj.m_Ry(1))                           % Option 3: Centred at 180 deg y-rotation... [Conditions ensure |y| > 0.5, WLOG choose the sign y > 0.5)
		        r = sqrt(1.0 - (obj.m_Ry(1) - obj.m_Ry(5) + obj.m_Ry(9)));    % = 2*|y|
		        s = 0.5/r;                                        % = 1/(4*|y|)
		        obj.m_Qy(1) = s*(obj.m_Ry(3) - obj.m_Ry(7));                  % = (4yw)/(4*|y|) = w*sgn(y) = w
		        obj.m_Qy(2) = s*(obj.m_Ry(4) + obj.m_Ry(2));                  % = (4xy)/(4*|y|) = x*sgn(y) = x
		        obj.m_Qy(3) = 0.5*r;                                  % = |y|           = y*sgn(y) = y
		        obj.m_Qy(4) = s*(obj.m_Ry(8) + obj.m_Ry(6));                  % = (4yz)/(4*|y|) = z*sgn(y) = z
	        else                                                  % Option 4: Centred at 180 deg x-rotation... (Conditions ensure |x| > 0.5, WLOG choose the sign x > 0.5)
	        
		        r = sqrt(1.0 + (obj.m_Ry(1) - obj.m_Ry(5) - obj.m_Ry(9)));    % = 2*|x|
		        s = 0.5/r;                                        % = 1/(4*|x|)
		        obj.m_Qy(1) = s*(obj.m_Ry(8) - obj.m_Ry(6));                  % = (4xw)/(4*|x|) = w*sgn(x) = w
		        obj.m_Qy(2) = 0.5*r;                                  % = |x|           = x*sgn(x) = x
		        obj.m_Qy(3) = s*(obj.m_Ry(4) + obj.m_Ry(2));                  % = (4xy)/(4*|x|) = y*sgn(x) = y
		        obj.m_Qy(4) = s*(obj.m_Ry(3) + obj.m_Ry(7));                  % = (4xz)/(4*|x|) = z*sgn(x) = z
            end
        
	        % Any deviations from being a unit quaternion (that might be experienced here due to the inaccuracies
	        % of floating point arithmetic) are pretty much irrelevant. This is firstly because they will only
	        % ever be extremely minor eps deviations, given the mathematical correctness of this algorithm, but also
	        % because any scaling in Qy is swallowed up by the Kp in the expression for m_w in the update()
	        % function anyway. This is why no quaternion normalisation step has been added here.
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function   resetLambda(obj) 
            obj.setLambda(0); % Restarts (activates) quick learning by setting &lambda; to zero.
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function   setLambda(obj,value) 
            % Sets &lambda; to the desired value, by default this is `1.0`, which turns quick learning off. Values outside of `[0,1]` are coerced.
            if value>=1
                value=1;
            elseif value<=0
                value=0;
            end
            obj.m_lambda=value;
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function reset(obj, quickLearn, resetGyroBias) % Note: Resets the entire class, except for the magnetometer calibration, the acc-only resolution method, and the configuration variables (i.e. the PI gains and the quick learn time)
	        % Reset the estimator state
	        obj.resetState(resetGyroBias);        
	        % Reset the lambda value to reinitiate quick learning (if requested)
	        if(quickLearn) 
                obj.resetLambda();
            else 
                obj.setLambda(1);
            end
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function resetAll(obj, quickLearn) % Note: Resets absolutely everything about the class
	        % Initialise the acc-only resolution method
	        obj.setAccMethod(obj.ME_DEFAULT);
        
	        % Initialise the configuration variables
	        obj.setPIGains(2.20, 2.65, 10.0, 1.25);
	        obj.setQLTime(3.0);
        
	        % Initialise the magnetometer calibration
	        obj.setMagCalib(1.0, 0.0, 0.0);
        
	        % Reset the attitude estimator
	        obj.reset(quickLearn,true);
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function resetState(obj,resetGyroBias) % Note: Currently used by reset() only
	        % Declare variables
	        i;	        
	        % Initialise the attitude estimate
	        obj.setAttitude(1.0, 0.0, 0.0, 0.0); % Resets m_Qhat, m_eulerValid and m_fusedValid internally        
	        % Update the alternative attitude estimate representations
	        obj.updateEuler(); % Resets m_Ehat and m_eulerValid internally
	        obj.updateFused(); % Resets m_Fhat, m_FhatHemi and m_fusedValid internally        
	        % Initialise the gyro bias estimate
	        if(resetGyroBias)
		        obj.setGyroBias(0, 0, 0); % Resets m_bhat internally
            end
	        % Initialise the remaining size 3 internal variables
		        obj.m_w = [0 0 0];
		        obj.m_wold = [0 0 0];
		        obj.m_omega = [0 0 0];
		        obj.m_base = [0 0 0];        
	        % Initialise the remaining size 4 internal variables
		        m_Qy = [1 0 0 0];
		        m_Qtilde = [1 0 0 0];
		        m_dQ = [0 0 0 0];
		        m_dQold = [0 0 0 0];
	        % Initialise the remaining size 9 internal variables
		        m_Ry = [1 0 0 0 1 0 0 0 1];
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function setAccMethod(obj, method) 
             if(method < obj.ME_DEFAULT || method >= obj.ME_COUNT) % Sets the acc-only measurement resolution method to use.
                     obj.m_accMethod=method;
             end
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function setPIGains(obj, Kp,  Ti,  KpQuick,  TiQuick)
	        % Set the standard PI gains
	        if((Kp > 0.0) && (Ti > 0.0))
		        obj.m_Kp = Kp;
		        obj.m_Ti = Ti;
	        end
        
	        % Set the quick learning PI gains
	        if((KpQuick > 0.0) && (TiQuick > 0.0))
		        obj.m_KpQuick = KpQuick;
		        obj.m_TiQuick = TiQuick;
	        end
        end
        % -----------------------------------------------------------------------------------------------------------------------
        % 
        % -----------------------------------------------------------------------------------------------------------------------
        function   setQLTime(obj, QLTime) 
            if QLTime > 0.0 
                obj.m_QLTime = QLTime;% Sets the quick learning time to use. See `getQLTime()` for more details (Note: Non-positive values of `QLTime` are ignored by this function).
            end
        end
    
    end
end

    