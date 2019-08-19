clear all
close all
wB = load('DLCwithBeta.mat');
woB = load('DLCwithoutBeta.mat');

LW = 3;
FS = 20;
aspectRatioW = [16 10]*70;
aspectRatioN = [8 10]*70;

w = 1.25;
a = max(w)*2*1.1+0.25;
b = max(w)*2+1;

marks = [0,a/2;
         0,-a/2;
         5,a/2;
         5,-a/2;
         10,a/2;
         10,-a/2;
         23.5,a/2+1
         23.5,a/2+1+b
         29,a/2+1
         29,a/2+1+b
         34.5,a/2+1
         34.5,a/2+1+b
         47,+1.5
         47,-1.5
         53,+1.5
         53,-1.5
         59,+1.5
         59,-1.5];

[~,t0] = min(abs(wB.X(2:end)));
t0 = wB.t(t0);

%% Plot yaw rate and reference with Beta = 1
xlim = [marks(1)-5 marks(end,1)+5];
% ylim = [min(wB.Y) max(wB.Y)];

figure('Position',[0,0, aspectRatioW]);
hTrajectory1 = plot(wB.X,wB.Y+10,'r--');
hold on
hTrajectory2 = plot(woB.X,woB.Y-10,'k--');
hCones1  = plot(marks(:,1),marks(:,2)-10,'o');
hCones2  = plot(marks(:,1),marks(:,2)+10,'o');

axis equal
set(hCones1,'Color',[0    0.4470    0.7410])
set(hCones2,'Color',[0    0.4470    0.7410])
set(gca,'XLim',xlim)
set(get(gca,'Children'),'LineWidth',LW)
xlabel('Distance [m]')
ylabel('Distance [m]')
legend('Yaw-Beta controller','Yaw controller')


set(gca,'FontSize',FS)
set(gca,'LineWidth',LW)

%%
figure('Position',[0,0, aspectRatioW]);

hBeta1 = plot(wB.t - t0,atan2(wB.v,wB.u)*180/pi,'ro-');
hold on
hBeta2 = plot(woB.t - t0,atan2(woB.v,woB.u)*180/pi,'kv-');

set(gca,'xlim',[-1 4.5])
set(get(gca,'Children'),'LineWidth',LW)
set(hBeta1,'MarkerIndices',[(25:50:325), (335:15:675), (725:50:length(wB.u))])
set(hBeta2,'MarkerIndices',[(1:50:300), (310:15:650), (700:50:length(wB.u))])
set(get(gca,'Children'),'MarkerSize',10)
set(gca,'FontSize',FS)
set(gca,'LineWidth',LW)
xlabel('Time [s]')
ylabel('Side slip angle [degree]')

legend('Yaw-Beta controller','Yaw controller')



