function [AHRS,error,parameters,reoptimIters,reoptimParam]=optimFilterParams(AHRS,RMS,dataset,reoptimIters,reoptimParam,IMU)

close all; % close all figures

%% 
set=dataset;   %set1=1 set2=2 set12=3 set123=4

if(set<5)
    load('Justa_dataset.mat')
elseif(set==5)
    load('ALS_dataset.mat')
elseif(set==6)
    load('Synthetic2.mat')
elseif(set==7)
    load('Synthetic3.mat')
end

switch(set)
    case 1
        start=1;
        myEnd=2800-1;
    case 2
        start=2800;
        myEnd=5300-1;
    case 3
        start=1;
        myEnd=5300-1;
    otherwise
        start=1;
        myEnd=length(time)-1;
end
optimA=0;
optimB=0;
optimC=0;
className=class(AHRS);
switch(className)
    case 'MadgwickAHRS3'
        params=1;
        optimA=AHRS.Beta;
    case 'Wilson_Madgwick_AHRS'
        params=1;
        optimA=AHRS.Beta;            
    case 'MadgwickAHRSclanek'
        params=1;
        optimA=AHRS.Beta;
    case 'JustaAHRSPureFast'
        params=3;
        optimA=AHRS.gain;
        optimB=AHRS.wAcc;
        optimC=AHRS.wMag;
    case 'JustaAHRSPure'
        params=2;
        optimA=AHRS.wAcc;
        optimB=AHRS.wMag;
    case 'JustaAHRSPureFastLinearCorr'
        params=3;
        optimA=AHRS.gain;
        optimB=AHRS.wAcc;
        optimC=AHRS.wMag;
    case 'JustaAHRSPureFastConstantCorr'
        params=2;
        optimA=AHRS.wAcc;
        optimB=AHRS.wMag;
    case 'JustaAHRSFunctionMultiParam'
        params=10;
        optimA=AHRS.wAcc;
        optimB=AHRS.wMag;
    case 'Valenti_AHRS'
        params=2;
        optimA= AHRS.wAcc;
        optimB= AHRS.wMag;
    case 'AdmirallWilsonAHRS'
        params=1;
        optimA= AHRS.Beta;
    case 'JinWuKF_AHRSreal2'
        params=2;
        optimA= AHRS.Sigma_a(1,1);
        optimB= AHRS.Sigma_m(1,1);
    case 'YoungSooSuh_AHRS'
        params=3;
        optimA= AHRS.rg;
        optimB= AHRS.ra;
        optimC= AHRS.rm;        
end

for sw=1:params
    
    switch sw
        case 1
            localA=[optimA*0.3,optimA*0.6,optimA*0.9,optimA*0.95,optimA*1.05,optimA*1.1,optimA*1.4,optimA*1.7,rand()*optimA*2];
            %                     localA(localA>1) = 1;
            localA(localA<0) = 0;
        case 2
            localB=[optimB*0.3,optimB*0.6,optimB*0.9,optimB*0.95,optimB*1.05,optimB*1.1,optimB*1.4,optimB*1.7,rand()*optimB*2];
            %                     localB(localB>1) = 1;
            localB(localB<0) = 0;
        case 3
            localC=[optimC+0.02,optimC*0.6,optimC*0.9,optimC*0.95,optimC*1.05,optimC*1.1,optimC*1.4,optimC+0.02,rand()*optimC*2];
            %                     localC(localC>1) = 1;
            localC(localC<0) = 0;
    end
    
    scalErr=zeros(5,9);
    
    for bet = 1:9
        
        parameter=zeros(1,3);
        switch sw
            case 1
                
                parameter(1)=localA(bet);
                parameter(2)=optimB;
                parameter(3)=optimC;
            case 2
                parameter(1)=optimA;
                parameter(2)=localB(bet);
                parameter(3)=optimC;
            case 3
                parameter(1)=optimA;
                parameter(2)=optimB;
                parameter(3)=localC(bet);
        end
        
        switch(className)
            case 'MadgwickAHRS3'
                AHRS.Beta=parameter(1);
            case 'Wilson_Madgwick_AHRS'
                AHRS.Beta=parameter(1);                
            case 'MadgwickAHRSclanek'
                AHRS.Beta=parameter(1);
            case 'JustaAHRSPureFast'
                AHRS.gain=parameter(1);
                AHRS.wAcc=parameter(2);
                AHRS.wMag=parameter(3);
            case 'JustaAHRSPure'
                AHRS.wAcc=parameter(1);
                AHRS.wMag=parameter(2);
            case 'Valenti_AHRS'
                AHRS.wAcc=parameter(1);
                AHRS.wMag=parameter(2);
            case 'AdmirallWilsonAHRS'
                AHRS.Beta=parameter(1);               
            case 'JustaAHRSPureFastLinearCorr'
                AHRS.gain=parameter(1);
                AHRS.wAcc=parameter(2);
                AHRS.wMag=parameter(3);
            case 'JustaAHRSPureFastConstantCorr'
                AHRS.wAcc=parameter(1);
                AHRS.wMag=parameter(2);
            case 'JinWuKF_AHRSreal2'
                AHRS.Sigma_g=diag([1 1 1]);
                AHRS.Sigma_a=diag([1 1 1])*parameter(1);
                AHRS.Sigma_m=diag([1 1 1])*parameter(2);
            case 'YoungSooSuh_AHRS'
                AHRS.rg=parameter(1);
                AHRS.ra=parameter(2);
                AHRS.rm=parameter(3);
        end
        
        AHRS.Quaternion = qViconReference(start,:);
        
        quaternionCountJ = zeros(myEnd, 4);
        
        for t = start:myEnd
            if(t==start)
                dt=time(1);
            else
                dt=time(t)-time(t-1);
            end

            AHRS.SamplePeriod=dt;
            if(IMU)
                AHRS.UpdateIMU(Gyroscope(t,:), Accelerometer(t,:));
            else
                AHRS.Update(Gyroscope(t,:), Accelerometer(t,:),Magnetometer(t,:));
            end
            quaternionCountJ(t, :) = AHRS.Quaternion;
            
        end
        %             qErr=quaternProd([qViconReference(start:3450,:);qViconReference(3550:myEnd,:)],quaternConj([quaternionCountJ(start:3450,:);quaternionCountJ(3550:myEnd,:)]));
        qErr=quaternProd(qViconReference((start:myEnd),:),quaternConj(quaternionCountJ(start:myEnd,:)));
        qErr(qErr(:,1)<0,:)=-qErr(qErr(:,1)<0,:);
        if(IMU)
%             filEul=real(quat2eul(qErr));
%             filEul(:,[1 3]) = filEul(:,[3 1]);
%             uhel=abs(sqrt(sum(filEul(:,1:2).^2,2))*180/pi);
            uhel=abs(2*atan2(sqrt(sum(qErr(:,2:4).^2,2)),qErr(:,1))*180/pi-0.8);
        else
            uhel=abs(2*atan2(sqrt(sum(qErr(:,2:4).^2,2)),qErr(:,1))*180/pi-0.8); %-0.8
        end
        
        if(RMS==1)
            uhel=uhel.^2;
        end
        
        scalErr(5,bet)=mean(uhel);
        
    end
    
    [minim,ind]= min(scalErr(5,:));
    
    switch sw
        case 1
            optimA=localA(ind);
        case 2
            optimB=localB(ind);
        case 3
            optimC=localC(ind);
    end
    
end

switch(className)
    case 'MadgwickAHRS3'
        AHRS.Beta=optimA;
    case 'Wilson_Madgwick_AHRS'
        AHRS.Beta=optimA;        
    case 'MadgwickAHRSclanek'
        AHRS.Beta=optimA;
    case 'JustaAHRSPureFast'
        AHRS.gain=optimA;
        AHRS.wAcc=optimB;
        AHRS.wMag=optimC;
    case 'JustaAHRSPure'
        AHRS.wAcc=optimA;
        AHRS.wMag=optimB;
    case 'JustaAHRSPureFastLinearCorr'
        AHRS.gain=optimA;
        AHRS.wAcc=optimB;
        AHRS.wMag=optimC;
    case 'JustaAHRSPureFastConstantCorr'
        AHRS.wAcc=optimA;
        AHRS.wMag=optimB;
    case 'Valenti_AHRS'
        AHRS.wAcc=optimA;
        AHRS.wMag=optimB;
    case 'AdmirallWilsonAHRS'
        AHRS.Beta=optimA;     
    case 'JinWuKF_AHRSreal2'
        AHRS.Sigma_a=diag([1 1 1])*optimA;
        AHRS.Sigma_m=diag([1 1 1])*optimB;
    case 'YoungSooSuh_AHRS'
        AHRS.rg=optimA;
        AHRS.ra=optimB;
        AHRS.rm=optimC;
end
parameters=[optimA,optimB,optimC];
error=minim;

%% End of script