classdef YoungSooSuh_AHRS < handle


    %% Public properties
    properties (Access = public)
        SamplePeriod = 1/256;
        Quaternion = [1 0 0 0];     % output quaternion describing the Earth relative to the sensor
        
        q_err=[0 0 0];
        
        rg=0.317;
        ra=0.0004156;%0.0056;
        rm=0.00057;%0.001
        
        alf=0;
        bet=0;
        gam=0;
        
        kAlf=0;
        kBet=0;
        
        iter=0;
        
        test=[0 0 0 0];
        test2=[0 0 0 0];
    end

    %% Public methods
    methods (Access = public)
        function obj = YoungSooSuh_AHRS(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'), obj.SamplePeriod = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Quaternion'), obj.Quaternion = varargin{i+1};
%                 elseif  strcmp(varargin{i}, 'Beta'), obj.Beta = varargin{i+1};
%                 elseif  strcmp(varargin{i}, 'alf'), obj.alf = varargin{i+1};
                else error('Invalid argument');
                end
            end
        end
        function obj = Update(obj, Gyroscope, Accelerometer, Magnetometer)
            g=1;
            
            dt=obj.SamplePeriod;
            q=obj.Quaternion;

            q_dif_est=0.5*quaternProd(q,[0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]);            
            q_est=q+q_dif_est*dt;
                        
            AccNorm=Accelerometer./norm(Accelerometer);
            Magnetometer=Magnetometer./norm(Magnetometer);

            cosAccMag=dot(AccNorm,Magnetometer);
            sinAccMag=sqrt(1-cosAccMag^2);
            sinDipAng=cosAccMag;
            cosDipAng=sinAccMag;
            
            ak_minus=obj.alf+obj.rg*dt^2/4;
            bk_minus=obj.bet+obj.rm*dt^2/4;
            
            obj.kAlf=2*ak_minus*g/(4*ak_minus*g^2+obj.ra);
            obj.kBet=-2*(bk_minus*cosDipAng-4*obj.gam*sinDipAng)/...
                (4*ak_minus*sinDipAng^2+4*bk_minus*cosDipAng^2-8*obj.gam*cosDipAng*sinDipAng+obj.rm);
            
            obj.alf=(4*ak_minus*g^2+obj.ra)*obj.kAlf^2-4*ak_minus*g*obj.kAlf+ak_minus;
            obj.bet=(4*ak_minus*sinDipAng^2+4*bk_minus*cosDipAng^2+obj.rm-8*obj.gam*cosDipAng*sinDipAng)*obj.kBet^2 ...
                        +4*(bk_minus*cosDipAng-4*obj.gam*sinDipAng)*obj.kBet+bk_minus;    
            obj.gam=-(2*g*obj.kAlf-1)*(obj.gam+2*(obj.gam*cosDipAng-ak_minus*sinDipAng)*obj.kBet);

            aRot=quaternProd(q_est,quaternProd([0 Accelerometer],quaternConj(q_est)));
            z1=(aRot(2:4)-[0 0 g]);
            z1=z1(1:2)';
            
            mRot=quaternProd(q_est,quaternProd([0 Magnetometer],quaternConj(q_est)));
            z2=mRot(3);
            
            K=[0 obj.kAlf 0; -obj.kAlf 0 0; 0 0 obj.kBet];
            H=[0 -1 0; 1 0 0; sinDipAng, 0 -cosDipAng]*2;
            obj.q_err=obj.q_err+(K*([z1;z2]-H*obj.q_err'))';
            
%             obj.q_err=[0 0 0];
            obj.test = [z1;z2;0]';
            obj.test2 = [aRot(2:4) 0];

            q=quaternProd([1 obj.q_err],q_est);

            obj.Quaternion = q / norm(q); % normalise quaternion
        end
    end
end