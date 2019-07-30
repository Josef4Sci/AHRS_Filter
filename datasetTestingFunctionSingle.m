function [M1,uhelM1,tOut]=datasetTestingFunctionSingle(AHRS1,dataset,RMS,IMU,ExceptPeak)

%addpath('quaternion_library');      % include quaternion library
close all;                          % close all figures
% clear;                              % clear all variables
% clc;                                % clear the command terminal

set=dataset;   %set1=1 set2=2 set12=3 set123=4 ALSdataset=5

if(set<5)
    load('Justa_dataset.mat')
elseif(set==5)
    load('ALS_dataset.mat')
end
tOut=time;

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
    case 4
        start=1;
        myEnd=length(time)-1;
    case 5
        start=1;
        myEnd=length(time)-1;
end

%%

AHRS1.Quaternion = qViconReference(start,:);

quaternionCountM1 = zeros(myEnd, 4);


for t = start:myEnd
    if(t==start)
        dt=time(1);
    else
        dt=time(t)-time(t-1);
    end
    if(t==1659)
        PAUS=1;
    end
    AHRS1.SamplePeriod=dt;
    
    if(IMU)
        AHRS1.UpdateIMU(Gyroscope(t,:), Accelerometer(t,:));
    else
        AHRS1.Update(Gyroscope(t,:), Accelerometer(t,:),Magnetometer(t,:));
    end
    quaternionCountM1(t, :) = AHRS1.Quaternion/norm(AHRS1.Quaternion);
end

qErr=quaternProd(qViconReference(start:myEnd,:),quaternConj(quaternionCountM1(start:myEnd,:)));
qErr(qErr(:,1)<0,:)=-qErr(qErr(:,1)<0,:);

if(~ExceptPeak)
    if(IMU)
        filEul=real(quat2eul(qErr));
        filEul(:,[1 3]) = filEul(:,[3 1]);
        uhelM1=abs(sqrt(sum(filEul(:,1:2).^2,2))*180/pi);
    else
        uhelM1=abs(2*atan2(sqrt(sum(qErr(:,2:4).^2,2)),qErr(:,1))*180/pi);
    end
else
    if(set==1||set==5)
        uhelM1=123456789;
    else
        if(IMU)
            filEul=real(quat2eul([qErr(1:4503,:);qErr(5254:length(qErr(:,1)),:)]));
            filEul(:,[1 3]) = filEul(:,[3 1]);
            uhelM1=abs(sqrt(sum(filEul(:,1:2).^2,2))*180/pi);
        else
            uhelM1=abs(2*atan2(sqrt(sum([qErr(1:4503,2:4);qErr(5254:length(qErr(:,1)),2:4)].^2,2)),[qErr(1:4503,1);qErr(5254:length(qErr(:,1)),1)])*180/pi);
        end
    end
end

if(RMS==1)
    uhelM1=uhelM1.^2;
end

M1=mean(uhelM1);

end