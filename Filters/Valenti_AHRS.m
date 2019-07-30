classdef Valenti_AHRS < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        SamplePeriod = 1/256;
        Quaternion = [1 0 0 0];     % output quaternion describing the Earth relative to the sensor
        test=[0 0 0 0];
        test2=[0 0 0 0];
        wAcc=0.01;
        wMag=0.01;
        gain=0;
    end
    
    methods (Access = public)
        function obj = Valenti_AHRS(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'), obj.SamplePeriod = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Quaternion'), obj.Quaternion = varargin{i+1};
                else error('Invalid argument');
                end
            end;
        end
        function obj = Update(obj, Gyroscope, Accelerometer, Magnetometer)
            q = obj.Quaternion; % short name local variable for readability

            % Normalise accelerometer measurement
            if(norm(Accelerometer) == 0), return; end	% handle NaN
            acc = Accelerometer / norm(Accelerometer);	% normalise magnitude

            % Normalise magnetometer measurement
            if(norm(Magnetometer) == 0), return; end	% handle NaN
            mag = Magnetometer / norm(Magnetometer);	% normalise magnitude

            qDot=0.5 * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]);
            quatGyrPred= q + qDot*obj.SamplePeriod;
            qPred=quatGyrPred/norm(quatGyrPred);

%             q0_pred = q(1) + 0.5*obj.SamplePeriod*(-Gyroscope(1)*q(2) - Gyroscope(2)*q(3) - Gyroscope(3)*q(4));
%             q1_pred = q(2) + 0.5*obj.SamplePeriod*(+Gyroscope(1)*q(1) + Gyroscope(2)*q(4) - Gyroscope(3)*q(3));
%             q2_pred = q(3) + 0.5*obj.SamplePeriod*(-Gyroscope(1)*q(4) + Gyroscope(2)*q(1) + Gyroscope(3)*q(2));
%             q3_pred = q(4) + 0.5*obj.SamplePeriod*(+Gyroscope(1)*q(3) - Gyroscope(2)*q(2) + Gyroscope(3)*q(1));
%             
%             qPred=[q0_pred,q1_pred,q2_pred,q3_pred];

            x=acc(1);
            y=acc(2);
            z=acc(3);
            qr=qPred;
            vx = (qr(1)*qr(1) + qr(2)*qr(2) - qr(3)*qr(3) - qr(4)*qr(4))*x + 2*(qr(2)*qr(3) - qr(1)*qr(4))*y + 2*(qr(2)*qr(4) + qr(1)*qr(3))*z;
            vy = 2*(qr(2)*qr(3) + qr(1)*qr(4))*x + (qr(1)*qr(1) - qr(2)*qr(2) + qr(3)*qr(3) - qr(4)*qr(4))*y + 2*(qr(3)*qr(4) - qr(1)*qr(2))*z;
            vz = 2*(qr(2)*qr(4) - qr(1)*qr(3))*x + 2*(qr(3)*qr(4) + qr(1)*qr(2))*y + (qr(1)*qr(1) - qr(2)*qr(2) - qr(3)*qr(3) + qr(4)*qr(4))*z;

%             obj.test=[vx vy vz 0];
            
            dq0 =  sqrt((vz + 1) * 0.5);
            dqa = [dq0, vy/(2.0 * dq0), -vx/(2.0 * dq0), 0];            
            dqa = obj.scaleQuaternion(obj.wAcc,dqa);
            
            q=quaternProd(qPred, dqa);
            
            x=mag(1);
            y=mag(2);
            z=mag(3);
            qr=q;
            vx = (qr(1)*qr(1) + qr(2)*qr(2) - qr(3)*qr(3) - qr(4)*qr(4))*x + 2*(qr(2)*qr(3) - qr(1)*qr(4))*y + 2*(qr(2)*qr(4) + qr(1)*qr(3))*z;
            vy = 2*(qr(2)*qr(3) + qr(1)*qr(4))*x + (qr(1)*qr(1) - qr(2)*qr(2) + qr(3)*qr(3) - qr(4)*qr(4))*y + 2*(qr(3)*qr(4) - qr(1)*qr(2))*z;
            vz = 2*(qr(2)*qr(4) - qr(1)*qr(3))*x + 2*(qr(3)*qr(4) + qr(1)*qr(2))*y + (qr(1)*qr(1) - qr(2)*qr(2) - qr(3)*qr(3) + qr(4)*qr(4))*z;
            
            
            gamma = vx^2 + vy^2;
            beta = sqrt(gamma + vx*sqrt(gamma));
            dqm = [ beta / (sqrt(2.0 * gamma)), 0, 0, -vy / (sqrt(2.0) * beta)];
            
            dqm = obj.scaleQuaternion(obj.wMag,dqm);
            obj.test=dqm;
            quat=quaternProd(q, dqm);
%             quat=q;
%             obj.test=[vx vy vz beta];
            
%             if(quat(1)<0)
%                 quat=-quat;
%             end
            
            obj.Quaternion = quat/norm(quat);
        end
        
        function q = scaleQuaternion(obj,gain,quat)
        
            if (quat(1) < 0.0)     
                angle = acos(quat(1));
                A = sin(angle*(1.0 - gain))/sin(angle);
                B = sin(angle * gain)/sin(angle);
                quat(1) = A + B * quat(1);
                quat(2:4)=quat(2:4)*B;         
            else
                quat(1) = (1.0 - gain) + gain * quat(1);
                quat(2:4)=quat(2:4)*gain;
            end
            q=quat/norm(quat);
        end
        
        function obj = UpdateIMU(obj, Gyroscope, Accelerometer)
            q = obj.Quaternion; % short name local variable for readability

            % Normalise accelerometer measurement
            if(norm(Accelerometer) == 0), return; end	% handle NaN
            acc = Accelerometer / norm(Accelerometer);	% normalise magnitude

            qDot=0.5 * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]);
            quatGyrPred= q + qDot*obj.SamplePeriod;
            qPred=quatGyrPred/norm(quatGyrPred);

            x=acc(1);
            y=acc(2);
            z=acc(3);
            qr=qPred;
            vx = (qr(1)*qr(1) + qr(2)*qr(2) - qr(3)*qr(3) - qr(4)*qr(4))*x + 2*(qr(2)*qr(3) - qr(1)*qr(4))*y + 2*(qr(2)*qr(4) + qr(1)*qr(3))*z;
            vy = 2*(qr(2)*qr(3) + qr(1)*qr(4))*x + (qr(1)*qr(1) - qr(2)*qr(2) + qr(3)*qr(3) - qr(4)*qr(4))*y + 2*(qr(3)*qr(4) - qr(1)*qr(2))*z;
            vz = 2*(qr(2)*qr(4) - qr(1)*qr(3))*x + 2*(qr(3)*qr(4) + qr(1)*qr(2))*y + (qr(1)*qr(1) - qr(2)*qr(2) - qr(3)*qr(3) + qr(4)*qr(4))*z;

%             obj.test=[vx vy vz 0];
            
            dq0 =  sqrt((vz + 1) * 0.5);
            dqa = [dq0, vy/(2.0 * dq0), -vx/(2.0 * dq0), 0];            
            dqa = obj.scaleQuaternion(obj.wAcc,dqa);
            obj.test=dqa;
            q=quaternProd(qPred, dqa);

            obj.Quaternion = q/norm(q);
            
        end
    end
end

