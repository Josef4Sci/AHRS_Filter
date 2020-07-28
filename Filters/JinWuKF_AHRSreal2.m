classdef JinWuKF_AHRSreal2 < handle
% Guo, S.; Wu, J.; Wang, Z.; Qian, J. Novel MARG-Sensor Orientation Estimation Algorithm Using
% Fast Kalman Filter. Journal of Sensors 2017, Article ID 8542153 
% 
% Implementation Justa, based on https://github.com/zarathustr/FKF
    
    %% Public properties
    properties (Access = public)
        SamplePeriod = 1/256;
        Quaternion = [0.5 -0.5 -0.5 -0.5];%[1 0 0 0];     % output quaternion describing sensor coordinate frame to the Earth frame
        
        Sigma_g=1*eye(3);
        Sigma_a=3*eye(3);
        Sigma_m=500*eye(3);

        Pk=0.001*eye(4); %init value Pk

        q=[1;0;0;0];
    end

    %% Public methods
    methods (Access = public)
        function obj = JinWuKF_AHRSreal2(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'), obj.SamplePeriod = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Quaternion'), obj.Quaternion = varargin{i+1};

                else error('Invalid argument');
                end
            end
        end
        function obj = Update(obj, Gyroscope, Accelerometer, Magnetometer)
           
            dt=obj.SamplePeriod;
            qq=obj.Quaternion;
            q0=qq(1);
            q1=qq(2);
            q2=qq(3);
            q3=qq(4);

            wx=Gyroscope(1);
            wy=Gyroscope(2);
            wz=Gyroscope(3);
            
            Accelerometer=Accelerometer./norm(Accelerometer);
            Magnetometer=Magnetometer./norm(Magnetometer);

            mD=dot(Accelerometer,Magnetometer);
            mN=sqrt(1-mD^2);

            omega4=[0,-wx,-wy,-wz;
                wx,0,wz,-wy;
                wy,-wz,0,wx;
                wz,wy,-wx,0];

            Phi=eye(4)+dt/2*omega4;

            Dk=[q1 q2 q3;
                -q0 -q3 -q2;
                q2 -q0 -q1;
                -q2 q1 -q0];
            Xi=dt*dt/4*Dk*obj.Sigma_g*Dk';

            [qy, Jacob]=measurement_quaternion_acc_mag(Accelerometer,Magnetometer,[mN,0,mD], obj.Quaternion');
            qy=qy./norm(qy);

            Eps=Jacob*[obj.Sigma_a,zeros(3,3);zeros(3,3),obj.Sigma_m]*Jacob';

            q_=obj.Quaternion';
            Pk_ = obj.Pk;
            [qq , obj.Pk] = kalman_update(q_, qy, Pk_, Phi, Xi, Eps);

            qq=qq./norm(qq);

            obj.Quaternion = qq' / norm(qq); % normalise quaternion
        end
    end
end