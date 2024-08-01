
err=[];
errTr=[];
for i=1:100
    q=(rand(1,4)-0.5);
%     q=[2 1 0 0];
    q=q/norm(q);
     
%     mr=[rand(1,1), 0, rand(1,1)];
    mr=[3, 0, 1];
    ar=[0 0 1];
    m=quaternProd(quaternConj(q),quaternProd([0 mr], q));
    a=quaternProd(quaternConj(q),quaternProd([0 ar], q));
   
    m=m(2:4);
    a=a(2:4);
    
    qinit=[1,0,1,0];
    qinit=qinit/norm(qinit);
    qo=qinit;
    for j=1:100
        qo=JustaConstantFunction(qo, 0.01, [0.0,0.0,0.0], a, m,pi/10,pi/10);
    end
    for j=1:10
        qo=JustaConstantFunction(qo, 0.01, [0.0,0.0,0.0], a, m,pi/100,pi/100);
    end
    for j=1:10
        qo=JustaConstantFunction(qo, 0.01, [0.0,0.0,0.0], a, m,pi/1000,pi/1000);
    end

    if(q(1)<0)
            q=-q;
    end

    mr=[0.1, 0, 1];
    mr=mr/norm(mr);
    qTr=triad([a',(m/norm(m))'],[ar',mr']);
    if(qTr(1)<0)
            qTr=-qTr;
    end

%     qoquest=quest([a',m/norm(m)'],[mr';]);

    qErr=quaternProd(quaternConj(q),qo);
    uhel=abs(2*atan2(sqrt(sum(qErr(:,2:4).^2,2)),qErr(:,1))*180/pi);
    err=[err uhel];
    qErr=quaternProd(quaternConj(q),qTr);
    uhel=abs(2*atan2(sqrt(sum(qErr(:,2:4).^2,2)),qErr(:,1))*180/pi);
    errTr=[errTr uhel];
end
plot(errTr)
errO=mean(err)
errOtr=mean(errTr)