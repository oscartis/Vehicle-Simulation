% Postprocessing after the simulation. Generates plots of the relevant
% vehicle states and parameters. 

% x - [LongVel LatVel YawVel LongAcc LatAcc YawAcc]
close all
% Some figure formatting
set(0,'DefaultAxesFontSize',14)
set(0,'DefaultAxesLineWidth',2)
set(0,'defaultlinelinewidth',2)


% 
u       = x(1,:);
v       = x(2,:);
r       = x(3,:);
ax      = x(4,:);
ay      = x(5,:);
rDot    = x(6,:);


%% Vehicle travel animation
figure;

hsub1 = subplot(1,2,1);
hsub2 = subplot(1,2,2);
axes(hsub1);

subPos = get(hsub1,'Position');
set(hsub1,'Position',[subPos(1)*0.8 subPos(2), 0.7, subPos(4)]);


hTrajectory = plot(X(1),Y(1),'b--');
hTitle=title(sprintf('Time = %.1f',t(1)));
hold on;
hTrack = plot(trackPath(:,1),trackPath(:,2),'g--');
hAimPoint = plot([aimPoints(:,1,1),aimPoints(:,1,2)],'xb');
axis equal;
xlim([min(X)-10 max(X)+10])
ylim([min(Y)-10 max(Y)+10])
axis manual;


xlabel('X [m]')
ylabel('Y [m]')

l1t = l1*5;
l2t = l2*5;
wt = w*5;

X1 =  l1t*cos(Psi) - wt(1)*sin(Psi);
X2 =  l1t*cos(Psi) - wt(2)*sin(Psi);
X3 =  l2t*cos(Psi) - wt(4)*sin(Psi);
X4 =  l2t*cos(Psi) - wt(3)*sin(Psi);

Y1 =  wt(1)*cos(Psi) + l1t*sin(Psi);
Y2 =  wt(2)*cos(Psi) + l1t*sin(Psi);
Y3 =  wt(4)*cos(Psi) + l2t*sin(Psi);
Y4 =  wt(3)*cos(Psi) + l2t*sin(Psi);

hCar = patch([X1(1) X2(1) X3(1) X4(1)], [Y1(1) Y2(1) Y3(1) Y4(1)],'red','facecolor','none','edgecolor','red');

xLimits = get(gca,'xlim');
yLimits = get(gca,'ylim');
axis manual;
xlim(xLimits)
ylim(yLimits)

newPos = get(hsub1,'Position');
axes(hsub2);
hbar = bar(u(1)*3.6,'BarWidth',100,'FaceColor',[0.8 0.25 0.1]);
set(hsub2,'Position',[newPos(1)+newPos(3)+0.01, newPos(2), (1-(newPos(1)+newPos(3)+0.1)), newPos(4)]);
set(hsub2,'yLim',[0 80],'Ytick',(0:20:100));
set(hsub2,'Xtick',[])
set(hsub2,'Color',get(gcf,'Color'))
set(hsub2,'XColor',get(gcf,'Color'))
set(hsub2,'YAxisLocation','right')
hold on
hvref = plot([get(hsub2,'XLim')],[v_ref(1) v_ref(1)],'k');

for i = 1:1:length(Psi)
    set(hTrajectory,'XData',X(1:i),'YData',Y(1:i))
    set(hCar,'Xdata',X(i)+[X1(i) X2(i) X3(i) X4(i)], 'Ydata',Y(i)+[Y1(i) Y2(i) Y3(i) Y4(i)]);
    set(hAimPoint,'Xdata',aimPoints(1,i,:),'Ydata',aimPoints(2,i,:))
    set(hTitle,'String',sprintf('Time= %0.1f',t(i)));
    
    
    set(hbar,'YData',u(i)*3.6)
    set(hvref,'YData',[v_ref(i) v_ref(i)]*3.6)
    drawnow;
    
end

%% Vehicle path and orientation
% figure('Name','Vehicle path');
% plot(X,Y,'b--')
% axis equal;
% grid on;
% hold on;
% % Try to add markers on the plot representing the ESC test requirements
% % (Lateral stability requirements)
% xlabel('X [m]')
% ylabel('Y [m]')
% 
% for i = 1:1/(sampleTime):length(Psi)
%     patch(X(i)+[X1(i) X2(i) X3(i) X4(i)], Y(i)+[Y1(i) Y2(i) Y3(i) Y4(i)],'red','facecolor','none','edgecolor','red');
% end
% drawnow;

% % %% Vehicle lateral acceleration
% % figure;
% % plot(t,ay)
% % grid on;
% % xlabel('Time [s]')
% % ylabel('Lateral acceleration [m/s^2]')
% % 
% % %% Vehicle longitudinal acceleration
% % % figure;
% % % plot(t,ax)
% % % grid on;
% % % xlabel('Time [s]')
% % % ylabel('Longitudinal acceleration [m/s^2]')
% % 
% % %% Normal load on the wheels
% % % figure;
% % % plot(t,Fz)
% % % grid on;
% % % xlabel('Time [s]')
% % % ylabel('Wheel normal loads [N]')
% % % legend('Front left','Front right','Rear left','Rear right')