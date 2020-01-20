close all;

figure('DefaultAxesFontSize',18,'Position',[50,50,1000,600]);
hold on;
mml=40;
lineW=1.5;
start=1;
endPlot=2500;%length(M1uhel); %length(M1uhel)
if exist('M1uhel','var')
        plot(time(start:endPlot),movmean(M1uhel(start:endPlot),mml),'LineWidth',lineW);
end
if exist('M2uhel','var')
        plot(time(start:endPlot),movmean(M2uhel(start:endPlot),mml),'LineWidth',lineW);
end

if exist('M3uhel','var')
        plot(time(start:endPlot),movmean(M3uhel(start:endPlot),mml),'LineWidth',lineW);
end

if exist('M4uhel','var')
        plot(time(start:endPlot),movmean(M4uhel(start:endPlot),mml),'LineWidth',lineW);
end
hold off
legend('Proposed','Madgwick','Valenti','Guo')

ylabel('Squared angle error (deg^2)','FontSize',20);
xlabel('Time (s)','FontSize',20);

% mean(M1uhel([1:endPlot, 5299:length(M1uhel)]))
% mean(M2uhel(1:endPlot))
% mean(M3uhel(1:endPlot))
% mean(M4uhel([1:endPlot, 5299:length(M1uhel)]))