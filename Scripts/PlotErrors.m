close all;

plot_standard=0;
plot_madg=0;
plot_function=1;
if(plot_standard)
    % figure('DefaultAxesFontSize',18,'Position',[50,50,1000,600]);
    figure('DefaultAxesFontSize',18,'Position',[50,50,floor(1900*1/2),600]);
    hold on;
    mml=40;
    lineW=1.5;
    start=1;
    %length(M1uhel) 2500;%
    if exist('M1uhel','var')
        endPlot=length(M1uhel);
        plot(time(start:endPlot),movmean(M1uhel(start:endPlot),mml),'LineWidth',lineW);
    end
    if exist('M2uhel','var')
        endPlot=length(M2uhel);
        plot(time(start:endPlot),movmean(M2uhel(start:endPlot),mml),'LineWidth',lineW);
    end
    
    if exist('M3uhel','var')
        endPlot=length(M3uhel);
        plot(time(start:endPlot),movmean(M3uhel(start:endPlot),mml),'LineWidth',lineW);
    end
    
    if exist('M4uhel','var')
        plot(time(start:endPlot),movmean(M4uhel(start:endPlot),mml),'LineWidth',lineW);
    end
    axis manual
    if exist('M5uhel','var')
        p5= plot(time(start:endPlot),movmean(M5uhel(start:endPlot),mml),'LineWidth',lineW);
        p5.Color(4)=0.2;
        
        %figure(p5);
    end
    if(length(M1uhel)==6706)
           xline(70.92,'LineWidth',lineW);
           xline(39.3,'LineWidth',lineW);
    end
    if(length(M1uhel)==5299)
           xline(39.3,'LineWidth',lineW);
    end
    hold off
    legend('FSCF','Madgwick','Valenti','Guo','Suh')
end

if(plot_madg)
    figure('DefaultAxesFontSize',18,'Position',[50,50,floor(1900*1/2),600]);
    hold on;
    mml=40;
    lineW=1.5;
    start=1;
    %length(M1uhel) 2500;%
    if exist('M1uhel','var')
        endPlot=length(M1uhel);
        plot(time(start:endPlot),movmean(M1uhel(start:endPlot),mml),'LineWidth',lineW);
    end
    if exist('M2uhel','var')
        endPlot=length(M2uhel);
        plot(time(start:endPlot),movmean(M2uhel(start:endPlot),mml),'LineWidth',lineW);
    end
    legend('Madgwick','Linear Madgwick')
end

if(plot_function)
    figure('DefaultAxesFontSize',18,'Position',[50,50,floor(1900/2),600]);
    hold on;
    mml=40;
    lineW=1.5;
    start=1;
    %length(M1uhel) 2500;%
    if exist('M1uhel','var')
        endPlot=length(M1uhel);
        plot(time(start:endPlot),movmean(M1uhel(start:endPlot),mml),'LineWidth',lineW);
    end
    if exist('M2uhel','var')
        endPlot=length(M2uhel);
        plot(time(start:endPlot),movmean(M2uhel(start:endPlot),mml),'LineWidth',lineW);
    end
    
    if exist('M3uhel','var')
        endPlot=length(M3uhel);
        plot(time(start:endPlot),movmean(M3uhel(start:endPlot),mml),'LineWidth',lineW);
    end
    if(length(M1uhel)==6706)
           xline(70.92,'LineWidth',lineW);
           xline(39.3,'LineWidth',lineW);
    end
    if(length(M1uhel)==5299)
           xline(39.3,'LineWidth',lineW);
    end
    legend('Linear','Constant','Segmented')
end

% ylabel('Squared angle error (deg^2)','FontSize',20);
ylabel('Angle error (deg)','FontSize',20);
xlabel('Time (s)','FontSize',20);

% mean(M1uhel([1:endPlot, 5299:length(M1uhel)]))
% mean(M2uhel(1:endPlot))
% mean(M3uhel(1:endPlot))
% mean(M4uhel([1:endPlot, 5299:length(M1uhel)]))