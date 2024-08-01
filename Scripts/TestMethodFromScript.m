%% Start of script

close all;                          % close all figures
% clear;                              % clear all variables
% clc;                                % clear the command terminal
addpath("..\Filters\")
addpath("..\quaternion_library\")

%%
path="C:\Users\Pepa\Desktop\vrg\datasets\imus\take_3_cockpit_short\data.csv";
T = readtable(path,'NumHeaderLines',1);
Matrix=T{:,:};

%%
AHRS = JustaAHRSPureFastConstantCorr();
AHRS.wAcc=0.00001;%
AHRS.wMag=0.00;%
AHRS.wC=0.00001;%

% AHRS = MadgwickAHRS3('Beta',1.1);


AHRS.Quaternion=[1 0 0 0];
AHRS.Quaternion=AHRS.Quaternion/norm(AHRS.Quaternion);

%%

time=Matrix(:,1);
% Gyroscope=[-Matrix(:,4),-Matrix(:,2),Matrix(:,3)];
Gin=[Matrix(:,2),Matrix(:,3),Matrix(:,4)];
Ain=[Matrix(:,5),Matrix(:,6),Matrix(:,7)];

ax=[3,1,2];
Gyroscope=[-Gin(:,ax(1)),-Gin(:,ax(2)),Gin(:,ax(3))];
Accelerometer=[-Ain(:,3),-Ain(:,1),Ain(:,2)];


for t = 1:length(time)
    if(t==1)
       AHRS.SamplePeriod=0;
       for init = 1:1000
           aci=AHRS.wAcc;
           wci=AHRS.wC;
           AHRS.wAcc=0.001;%
           AHRS.wC=0.001;%
           AHRS.Update(Gyroscope(t,:), Accelerometer(t,:),[1 0 1]);
           AHRS.wAcc=aci;
           AHRS.wC=wci;
       end
    else
        AHRS.SamplePeriod=(time(t)-time(t-1))/1e9;
    end

    AHRS.Update(Gyroscope(t,:), Accelerometer(t,:),[1 0 1]);
    
    quaternionCountJ(t, :) = AHRS.Quaternion;
    
    %         if(CompareMethods==4 || CompareMethods==1 || CompareMethods==6)
                 test(t,:)=AHRS.test;
    %             test2(t,:)=AHRS.test2;
    %         end
end

%% 
qInitZero=quaternProd(quaternionCountJ,quaternConj(quaternionCountJ(1,:)));
subplot(2,1,1);
% qFixAxis=qInitZero()
tEst=(time-time(1))/1e9;
plot(tEst(tEst<350),qInitZero(tEst<350,:))

% figure
path="C:\Users\Pepa\Desktop\vrg\datasets\imus\take_3_cockpit_short\mcap.csv";
T = readtable(path,'NumHeaderLines',1);
Ref=T{:,5:8};
Ref2=quaternProd(Ref,quaternConj(Ref(1,:)));
Ref3=[Ref2(:,1),Ref2(:,4),Ref2(:,2),Ref2(:,3)];
subplot(2,1,2);
tref=(T{:,1}-time(1))/1e9;
plot(tref(tref<350&tref>0),Ref3(tref<350&tref>0,:))
beep