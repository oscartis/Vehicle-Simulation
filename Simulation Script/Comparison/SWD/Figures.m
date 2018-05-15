clear all
close all
wB = load('SWDWithB.mat');
woB = load('SWDWoB.mat');

LW = 3;
FS = 20;
aspectRatioW = [16 10]*70;
aspectRatioN = [8 10]*70;

%% Plot yaw rate and reference with Beta = 1

xlim = [-.2 3];
ylim = [-1.1 1.1];
t0 = 10;

figure('Position',[0,0, aspectRatioN]);
hYawRate = plot(wB.t-t0,wB.r,'k');
hold on
hYawRef  = plot(wB.t-t0,wB.r_ref(:,1),'--r');

set(gca,'XLim',xlim,'YLim',ylim)
set(get(gca,'Children'),'LineWidth',LW)
xlabel('Time [s]')
ylabel('Yaw Rate [rad/s]')

legend('Yaw rate','Yaw rate reference')

strSpeed = sprintf('Speed = %i km/h',round(mean(wB.u(100:end))*3.6));
text(xlim(1)+0.55*(xlim(2)-xlim(1)),ylim(1)+0.85*(ylim(2)-ylim(1)),strSpeed,'FontSize',FS)

set(gca,'FontSize',FS)



%% Plot r and rref with B = 0

figure('Position',[0,0, aspectRatioN]);
hYawRate = plot(woB.t-t0,woB.r,'k');
hold on
hYawRef  = plot(woB.t-t0,woB.r_ref(:,1),'--r');

set(gca,'XLim',xlim,'YLim',ylim)
set(get(gca,'Children'),'LineWidth',LW)
xlabel('Time [s]')
ylabel('Yaw Rate [rad/s]')

legend('Yaw rate','Yaw rate reference')

text(xlim(1)+0.55*(xlim(2)-xlim(1)),ylim(1)+0.85*(ylim(2)-ylim(1)),strSpeed,'FontSize',FS)

set(gca,'FontSize',FS)
