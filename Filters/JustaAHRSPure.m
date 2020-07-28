classdef JustaAHRSPure < handle
%   JUSTA, Josef; ŠMÍDL, Václav; HAMÁÈEK, Aleš. Fast AHRS Filter for Accelerometer, Magnetometer, 
%   and Gyroscope Combination with Separated Sensor Corrections. Sensors, 2020, 20.14: 3824.
%
%   Date          Author          Notes
%   30/5/2020     Josef Justa     Initial release
    
    properties
        SamplePeriod = 1/256;
        Quaternion = [1 0 0 0];     % output quaternion describing the Earth relative to the sensor
        Beta = 1;               	% algorithm gain
        
        test=[0 0 0 0];
        test2=[0 0 0 0];
        gain=0.0528152;
        wAcc=0.00248;
        wMag=1.35e-04;
    end
    
    methods (Access = public)
        function obj = JustaAHRSPure(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'), obj.SamplePeriod = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Quaternion'), obj.Quaternion = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Beta'), obj.Beta = varargin{i+1};
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
            
            wt=Gyroscope*obj.SamplePeriod/2;
%             qDotw=cos(wt(1))*cos(wt(2))*cos(wt(3))-sin(wt(1))*sin(wt(2))*sin(wt(3));
%             qDotx=cos(wt(1))*sin(wt(2))*sin(wt(3))+sin(wt(1))*cos(wt(2))*cos(wt(3));
%             qDoty=sin(wt(1))*cos(wt(2))*sin(wt(3))+cos(wt(1))*sin(wt(2))*cos(wt(3));
%             qDotz=sin(wt(1))*sin(wt(2))*cos(wt(3))+cos(wt(1))*cos(wt(2))*sin(wt(3));

%             qDotw=cos(wt(1))*cos(wt(2))*cos(wt(3))-sin(wt(1))*sin(wt(2))*sin(wt(3));
            qDotx=sin(wt(1));
            qDoty=sin(wt(2));
            qDotz=sin(wt(3));
            qDotw=sqrt(1-sumsqr([qDotx qDoty qDotz]));
            
            qDot=[qDotw qDotx qDoty qDotz];

            quatGyrPred= quaternProd(q,qDot);  %q + qDot*obj.SamplePeriod;
            
            qp=quatGyrPred;
            R=[2*(0.5 - qp(3)^2 - qp(4)^2)   0   2*(qp(2)*qp(4) - qp(1)*qp(3))
                2*(qp(2)*qp(3) - qp(1)*qp(4))  0  2*(qp(1)*qp(2) + qp(3)*qp(4))
                2*(qp(1)*qp(3) + qp(2)*qp(4))  0  2*(0.5 - qp(2)^2 - qp(3)^2)];
            
            ar=[0 0 1];
            accMesPred=(R*ar')';
            
%             alf=0.01;
%             obj.mr_z= (1-alf)*obj.mr_z + alf*dot(accMesPred,mag);MesPred

            mr_z= dot(accMesPred,mag);
            mr_x=sqrt(1-mr_z^2);
            mr=[mr_x 0 mr_z];
         
            magMesPred=(R*mr')';

%             ca = cross([0 0 1], accRefPred)/norm(cross([0 0 1], accRefPred))*obj.wAcc;
            ca=cross(acc,accMesPred);
%             da=dot(acc,accMesPred);
            vecA=ca/norm(ca);
            
            cm=cross(mag,magMesPred);
            vecB=cm/norm(cm);
            
            im=vecA*obj.wAcc/2+vecB*obj.wMag/2;
            im2=im*sinc(norm(im)/pi);
            qCor=[sqrt(1-sumsqr(im2)),im2];
            
            quat=quaternProd(quatGyrPred,qCor);
  
%             quat=quatGyrPred;
            if(quat(1)<0)
                quat=-quat;
            end
            
            obj.Quaternion = quat/norm(quat);
        end
        
        function obj = UpdateIMU(obj, Gyroscope, Accelerometer)
            q = obj.Quaternion; % short name local variable for readability
            
            % Normalise accelerometer measurement
            if(norm(Accelerometer) == 0), return; end	% handle NaN
            acc = Accelerometer / norm(Accelerometer);	% normalise magnitude
            
            wt=Gyroscope*obj.SamplePeriod/2;
            qDotx=sin(wt(1));
            qDoty=sin(wt(2));
            qDotz=sin(wt(3));
            qDotw=sqrt(1-sumsqr([qDotx qDoty qDotz]));
            
            qDot=[qDotw qDotx qDoty qDotz];

            quatGyrPred= quaternProd(q,qDot);  %q + qDot*obj.SamplePeriod;
            
            qp=quatGyrPred;
            
            ar=[0 0 1];
            
            R=[0 0  2*(qp(2)*qp(4) - qp(1)*qp(3))
               0 0  2*(qp(1)*qp(2) + qp(3)*qp(4))
               0 0  2*(0.5 - qp(2)^2 - qp(3)^2)];
            
            accMesPred=(R*ar')';
            
            ca=cross(acc,accMesPred);
            veca=ca/norm(ca);
            
            qCor=[1 veca*obj.wAcc/2];
            
            quat=quaternProd(qp,qCor);
            
            if(quat(1)<0)
                quat=-quat;
            end
            
            obj.Quaternion = quat/norm(quat);
        end
        
    end
    
end

