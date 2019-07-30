%%
uhel=zeros(4,length(gui.u1));
uhel(1,:)=gui.u1;        
uhel(2,:)=gui.u2;

%%
uhel(3,:)=gui.u1;
uhel(4,:)=gui.u2;

%%
% uhel=uhelALS;
disp(['Mean error method MadgModif: ', num2str(mean(uhel(1,:))),char(10)]);
disp(['Mean error method JustaFast: ', num2str(mean(uhel(2,:))),char(10)]);
disp(['Mean error method Valenti: ', num2str(mean(uhel(3,:))),char(10)]);
disp(['Mean error method Guo: ', num2str(mean(uhel(4,:))),char(10)]);
plot(movmean(uhel',200));
%%