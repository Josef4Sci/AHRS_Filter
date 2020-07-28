classdef MadgwickAHRSclanek < handle
%Justa Implementation of Madgwick's IMU and AHRS algorithms based on first
%part of paper (linear correction)

    %% Public properties
    properties (Access = public)
        SamplePeriod = 1/256;
        Quaternion = [1 0 0 0];     % output quaternion describing the Earth relative to the sensor
        Beta = 0.00166;               	% algorithm gain
        accR=[0 0 0 0];
    end

    %% Public methods
    methods (Access = public)
        function obj = MadgwickAHRSclanek(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'), obj.SamplePeriod = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Quaternion'), obj.Quaternion = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Beta'), obj.Beta = varargin{i+1};
                else error('Invalid argument');
                end
            end
        end
        function obj = Update(obj, Gyroscope, Accelerometer, Magnetometer)
            q = obj.Quaternion; % short name local variable for readability

            % Normalise accelerometer measurement
            if(norm(Accelerometer) == 0), return; end	% handle NaN
            Accelerometer = Accelerometer / norm(Accelerometer);	% normalise magnitude

            % Normalise magnetometer measurement
            if(norm(Magnetometer) == 0), return; end	% handle NaN
            Magnetometer = Magnetometer / norm(Magnetometer);	% normalise magnitude

           
            qStat=q;
            for i=1:20
                h = quaternProd(qStat, quaternProd([0 Magnetometer], quaternConj(qStat)));
                b = [0 norm([h(2) h(3)]) 0 h(4)];

                % Gradient decent algorithm corrective step
                F = [2*(qStat(2)*qStat(4) - qStat(1)*qStat(3)) - Accelerometer(1)
                    2*(qStat(1)*qStat(2) + qStat(3)*qStat(4)) - Accelerometer(2)
                    2*(0.5 - qStat(2)^2 - qStat(3)^2) - Accelerometer(3)
                    2*b(2)*(0.5 - qStat(3)^2 - qStat(4)^2) + 2*b(4)*(qStat(2)*qStat(4) - qStat(1)*qStat(3)) - Magnetometer(1)
                    2*b(2)*(qStat(2)*qStat(3) - qStat(1)*qStat(4)) + 2*b(4)*(qStat(1)*qStat(2) + qStat(3)*qStat(4)) - Magnetometer(2)
                    2*b(2)*(qStat(1)*qStat(3) + qStat(2)*qStat(4)) + 2*b(4)*(0.5 - qStat(2)^2 - qStat(3)^2) - Magnetometer(3)];
                J = [-2*qStat(3),                 	2*qStat(4),                    -2*qStat(1),                         2*qStat(2)
                    2*qStat(2),                 	2*qStat(1),                    	2*qStat(4),                         2*qStat(3)
                    0,                         -4*qStat(2),                    -4*qStat(3),                         0
                    -2*b(4)*qStat(3),               2*b(4)*qStat(4),               -4*b(2)*qStat(3)-2*b(4)*qStat(1),       -4*b(2)*qStat(4)+2*b(4)*qStat(2)
                    -2*b(2)*qStat(4)+2*b(4)*qStat(2),	2*b(2)*qStat(3)+2*b(4)*qStat(1),	2*b(2)*qStat(2)+2*b(4)*qStat(4),       -2*b(2)*qStat(1)+2*b(4)*qStat(3)
                    2*b(2)*qStat(3),                2*b(2)*qStat(4)-4*b(4)*qStat(2),	2*b(2)*qStat(1)-4*b(4)*qStat(3),        2*b(2)*qStat(2)];
                step = (J'*F)';
                step = step / norm(step);	% normalise step magnitude
                qStat=qStat-0.05*step;
            end
            % Compute rate of change of quaternion
            qGyr = q + 0.5 * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]) * obj.SamplePeriod;
            
            q= (obj.Beta)*qStat + (1-obj.Beta)* qGyr;
            
            obj.Quaternion = q / norm(q); % normalise quaternion
        end
        function obj = UpdateIMU(obj, Gyroscope, Accelerometer)
            q = obj.Quaternion; % short name local variable for readability

            % Normalise accelerometer measurement
            if(norm(Accelerometer) == 0), return; end	% handle NaN
            Accelerometer = Accelerometer / norm(Accelerometer);	% normalise magnitude

            % Gradient decent algorithm corrective step
            F = [2*(q(2)*q(4) - q(1)*q(3)) - Accelerometer(1)
                2*(q(1)*q(2) + q(3)*q(4)) - Accelerometer(2)
                2*(0.5 - q(2)^2 - q(3)^2) - Accelerometer(3)];
            J = [-2*q(3),	2*q(4),    -2*q(1),	2*q(2)
                2*q(2),     2*q(1),     2*q(4),	2*q(3)
                0,         -4*q(2),    -4*q(3),	0    ];
            step = (J'*F);
            step = step / norm(step);	% normalise step magnitude

            % Compute rate of change of quaternion
            qDot = 0.5 * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]) - obj.Beta * step';

            % Integrate to yield quaternion
            q = q + qDot * obj.SamplePeriod;
            obj.Quaternion = q / norm(q); % normalise quaternion
        end
    end
end