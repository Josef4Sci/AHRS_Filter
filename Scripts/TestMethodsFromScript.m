%% Start of script

close all;                          % close all figures
% clear;                              % clear all variables
% clc;                                % clear the command terminal

%%
CompareMethods=[1 2 3 6]; % 1=YoungSooSuh, 2=madg, 3=valenti, 4=PRCF, 5=MadgwickAHRSclanek, 6=modif2
set=6;   %set1=1 set2=2 set12=3 set123=4 ALSdataset=5 synthetic2=6 synthetic3=7

if(set<5)
    load('Justa_dataset.mat')
elseif(set==5)
    load('ALS_dataset.mat')
elseif(set==6)
    load('Synthetic2.mat')
elseif(set==7)
    load('Synthetic3.mat')
end

quaternionCountJ = zeros(length(time), 4);
test=zeros(length(time), 4);
test2=zeros(length(time), 4);

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


%%
AHRSs={};
if(any(CompareMethods==1))
    AHRS = YoungSooSuh_AHRS();
    AHRS.rg= 0.0370;
    AHRS.ra= 0.000031;
    AHRS.rm= 0.0000235;    
    AHRSs = cat(2,AHRSs,{AHRS});
end
%%
if(any(CompareMethods==2))
    switch(set)
        case 1
            AHRS = MadgwickAHRS3('Beta',0.011421);
        case 2
            AHRS = MadgwickAHRS3('Beta',0.03226);
        case 3
            AHRS = MadgwickAHRS3('Beta',0.016632);
        case 4
            AHRS = MadgwickAHRS3('Beta',0.02113);
        case 6
            AHRS = MadgwickAHRS3('Beta',0.0328);
    end
    AHRSs = cat(2,AHRSs,{AHRS});
end
%%
if(any(CompareMethods==3))
    AHRS = Valenti_AHRS();
    AHRS.wAcc=0.00116;%
    AHRS.wMag=1.012e-7;%
    switch(set)
        case 4
            AHRS.wAcc=0.00608;%
            AHRS.wMag=1.2759e-04;%
    end
    AHRSs = cat(2,AHRSs,{AHRS});
end
%%
if(any(CompareMethods==4))
    AHRS = JustaAHRSPureFast();
    AHRS.wAcc=0.001;%
    AHRS.wMag=0.01;%
    AHRSs = cat(2,AHRSs,{AHRS});
end
%%
if(any(CompareMethods==5))
    switch(set)
        case 1
            AHRS = MadgwickAHRSclanek('Beta',0.00166);
        otherwise
            AHRS = MadgwickAHRSclanek('Beta',0.01);
    end
    AHRSs = cat(2,AHRSs,{AHRS});
end
%%
if(any(CompareMethods==6))
    AHRS = JustaAHRSPureFastConstantCorr();
    AHRS.wAcc=0.000351;%
    AHRS.wMag=0.000294;%
    AHRSs = cat(2,AHRSs,{AHRS});
end

%%
scalErr=zeros(5,length(AHRSs));
names=[];
errorAngles=[];
for method = 1:length(AHRSs)
    names = [names,string(class(AHRSs{method}))];
    AHRSs{method}.Quaternion = qViconReference(start,:);
    quaternionCountJ = zeros(myEnd, 4);
    
    for t = start:myEnd
        %AHRS.iter=AHRS.iter+1;
        if(t==start)
            dt=time(1);
        else
            dt=time(t)-time(t-1);
        end
        AHRSs{method}.SamplePeriod=dt;
        
        AHRSs{method}.Update(Gyroscope(t,:), Accelerometer(t,:),Magnetometer(t,:));
        quaternionCountJ(t, :) = AHRSs{method}.Quaternion;
        
        %         if(CompareMethods==4 || CompareMethods==1 || CompareMethods==6)
        %             test(t,:)=AHRS.test;
        %             test2(t,:)=AHRS.test2;
        %         end
    end
    qErr=quaternProd(qViconReference((start:myEnd),:),quaternConj(quaternionCountJ(start:myEnd,:)));
    qErr(qErr(:,1)<0,:)=-qErr(qErr(:,1)<0,:);
    uhel=abs(2*atan2(sqrt(sum(qErr(:,2:4).^2,2)),qErr(:,1))*180/pi);
    errorAngles=[errorAngles uhel];
    %scalErr(4,method)=0;
    scalErr(5,method)=mean(uhel);
end


[minim2,ind]= min(scalErr(5,:));
disp(['Relative to the best:', num2str(scalErr(5,:)/minim2)]);

plot(time(2:end),movmean(errorAngles,300));
ylabel('MAE Error (deg)')
xlabel('Time (s)')
legend(names);
%% End of script