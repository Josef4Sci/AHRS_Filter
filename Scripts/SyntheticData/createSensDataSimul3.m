function [accD,magD,gyrD,quatern,time] = createSensDataSimul3()
clear;

%%params
inclination_angle=25.52;
mag_intensity=50e-3;

acc_scale_err=1e-2;
mag_scale_err=5e-2;
gyr_scale_err=0.01;

acc_noise_err=1e-2;
mag_noise_err=2e-4;
gyr_noise_err=5e-3;

gyr_off_err=1;  % °/s
mag_off_err=7e-4;  % T
acc_off_err=3e-3;  % G


% acc_scale_err=1e-2;
% mag_scale_err=5e-2;
% gyr_scale_err=0.01;
% 
% acc_noise_err=1e-2;
% mag_noise_err=2e-4;
% gyr_noise_err=0;
% 
% gyr_off_err=0;  % °/s
% mag_off_err=7e-4;  % T
% acc_off_err=3e-3;  % G

%%precount
acc_null=[0 0 1];
q_temp=[cosd(inclination_angle/2) 0 sind(inclination_angle/2) 0];
m=quaternProd(quaternConj(q_temp), quaternProd([0 acc_null], q_temp));
mag_null=m(2:4)*mag_intensity;
mag_null(1)=mag_null(1);

leng=12000;
generated_mag=zeros( leng,3);
generated_acc=zeros( leng,3);
generated_gyr=zeros( leng,3);

dt=0.0039;
time(1)=0;
quat(1,:)=[1 0 0 0];
angle=[linspace(0,1,2600) linspace(1,0,2600) zeros(1,8000)]*106.5*2*dt;%71
%%generate data

phase=1;

for i=1: leng
    if(phase==1&&time(i)>18&&time(i)<20&&abs(generated_acc(i-1,1))<0.01)
%         phase=2;
    end
    
    if(phase==1)        
        dir=[0.1 0.2 0.05];
        dir=dir/norm(dir);
        q_temp=[cosd(angle(i)/2) dir(1)*sind(angle(i)/2) dir(2)*sind(angle(i)/2) dir(3)*sind(angle(i)/2)];
        q_temp=q_temp/norm(q_temp);
        quat(i+1,:)=quaternProd(quat(i,:),q_temp);
        if(quat(i+1,1)<0)
            quat(i+1,:)=-quat(i+1,:);
        end
    end
    if(phase==2)    
        quat(i+1,:)=quat(i,:);
    end
    
    m=quaternProd(quaternConj(quat(i,:)), quaternProd([0 mag_null], quat(i,:)));
    generated_mag(i,:)=-(1+mag_scale_err)*m(2:4)+randn(1,3)*mag_noise_err+[mag_off_err -mag_off_err mag_off_err/2];
    a=quaternProd(quaternConj(quat(i,:)), quaternProd([0 acc_null], quat(i,:)));
    generated_acc(i,:)=(1+acc_scale_err)*a(2:4)+randn(1,3)*acc_noise_err+[-acc_off_err acc_off_err acc_off_err/2];
    
       
    time(i+1)=time(i)+dt;
    q=quaternProd(quaternConj(quat(i,:)),quat(i+1,:));
    if(q(1)<0)
        q=-q;
    end
    g(1)=q(2)*2/dt;
    g(2)=q(3)*2/dt;
    g(3)=q(4)*2/dt;
    generated_gyr(i,:)=((1+gyr_scale_err)*g*180/pi+randn(1,3)*gyr_noise_err+[-gyr_off_err gyr_off_err gyr_off_err/2])*pi/180;
    
end
intA=6000:8000;
generated_acc(intA,2)=generated_acc(intA,2)+0.5*sin(linspace(0,6*pi,2001))';

intM=9000:11000;
generated_mag(intM,2)=generated_mag(intM,2)+mag_intensity*0.5*sin(linspace(0,6*pi,2001))';

%
%   generated_mag(length(quat),:)=generated_mag(length(quat)-1,:);
%   generated_acc(length(quat),:)=generated_acc(length(quat)-1,:);

%%plot
time(length(time))=[];
quat(length(quat),:)=[];
subplot(4,1,1);
plot(time,generated_acc);
subplot(4,1,2);
plot(time,generated_mag);
subplot(4,1,3);
plot(time,generated_gyr);
subplot(4,1,4);
plot(time,quat);

accD=generated_acc;
magD=generated_mag;
gyrD=generated_gyr;
quatern=quat;
end