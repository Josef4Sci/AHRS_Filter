classdef Wilson_Madgwick_AHRS< handle
%Formulation of a new gradient descent MARG orientation algorithm:
%Case study on robot teleoperation
%
%   Date          Author          Notes
%   30/5/2020     Josef Justa     Initial release

    %% Public properties
    properties (Access = public)
        SamplePeriod = 1/256;
        Quaternion = [1 0 0 0];     % output quaternion describing the Earth relative to the sensor
        Beta=0.1;
        accR=[0 0 0 0];
        test=[0 0 0 0];
        test2=[0 0 0 0];
        iter=0;
    end

    %% Public methods
    methods (Access = public)
        function obj = Wilson_Madgwick_AHRS(varargin)
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
            
            % Reference direction of Earth's magnetic feild
            %h = quaternProd(q, quaternProd([0 Magnetometer], quaternConj(q)));
            %vrm = [norm([h(2) h(3)]) 0 h(4)];

            %%
            
            measureFused=cross(Accelerometer, Magnetometer);
            measureFused=measureFused/norm(measureFused);
            
            error=     [-2*(q(1)*q(4)+q(2)*q(3))- measureFused(1);
                        -q(1)^2 + q(2)^2 - q(3)^2 + q(4)^2 - measureFused(2);
                        -2*(q(3)*q(4)-q(1)*q(2) ) - measureFused(3)];
            J= -2*[
                -q(4) -q(1) q(2);
                -q(3) q(2) q(1);
                -q(2) -q(3) -q(4);
                -q(1) q(4) -q(3)];
             
            F = J*error;          
            
%%
            % Compute rate of change of quaternion
            qDot = 0.5 * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]) - obj.Beta * F';

            % Integrate to yield quaternion
            q = q + qDot * obj.SamplePeriod;%
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
            
            obj.test=- obj.Beta *step*obj.SamplePeriod;
            % Compute rate of change of quaternion
            qDot = 0.5 * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]) - obj.Beta * step';

            % Integrate to yield quaternion
            q = q + qDot * obj.SamplePeriod;
            obj.Quaternion = q / norm(q); % normalise quaternion
        end
    end
end