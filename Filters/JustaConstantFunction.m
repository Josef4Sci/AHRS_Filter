function q = JustaConstantFunction(q, dt, Gyroscope, Accelerometer, Magnetometer,wAcc,wMag)
        
        acc = Accelerometer / norm(Accelerometer);	% normalise magnitude
        
        mag = Magnetometer / norm(Magnetometer);	% normalise magnitude
        
        qDot=0.5 * dt * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]);
        qp= q + qDot;
        
        R=[2*(0.5 - qp(3)^2 - qp(4)^2)   0   2*(qp(2)*qp(4) - qp(1)*qp(3))
            2*(qp(2)*qp(3) - qp(1)*qp(4))  0  2*(qp(1)*qp(2) + qp(3)*qp(4))
            2*(qp(1)*qp(3) + qp(2)*qp(4))  0  2*(0.5 - qp(2)^2 - qp(3)^2)];
        
        % ar=[0 0 1];
        accMesPred=R(:,3)';

        ca=cross(acc,accMesPred);
        na=norm(ca);
        if(na~=0)
            veca=ca/na;
        else
            veca=[1 0 0];
        end

        phia=wAcc;        
       
        mr_z= dot(accMesPred,mag);
        mr_x=sqrt(1-mr_z^2);
        mr=[mr_x 0 mr_z];

        magMesPred=(R*mr')';
        

        cm=cross(mag,magMesPred);
        nm=norm(cm);
        if(nm~=0)
            vecm=cm/nm;
        else
            vecm=[1 0 0];
        end
        phim=wMag;
        
%         qCor=[1 veca*wAcc+vecm*wMag];
        im=veca*phia/2+vecm*phim/2;
        im2=im*sinc(norm(im)/pi);
        qCor=[sqrt(1-sumsqr(im2)),im2];
        
        q=quaternProd(qp,qCor);

        if(q(1)<0)
            q=-q;
        end
        q = q/norm(q);
    end
        