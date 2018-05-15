clear all
close all
wB = load('StepResponseWithB.mat');
woB = load('StepResponseWoB.mat');

LW = 3;
FS = 20;
aspectRatioW = [16 10]*70;
aspectRatioN = [8 10]*70;

%% Plot yaw rate and reference with Beta = 1
idx = find(wB.r_ref(:,1) > 0);
t0  = wB.t(idx(1));

yawRate90 = find(wB.r > 0.9*max(wB.r_ref(:,1)));
ref = mean(wB.r_ref(idx(1:end)));
interp1(wB.t,wB.r,wB.r(yawRate90(1)));
riseTime1 = interp1(wB.r(yawRate90(1)-1:yawRate90(1)),wB.t(yawRate90(1)-1:yawRate90(1)),ref*0.9)-t0;
xlim = [-.1 .3];
ylim = [-0.1 ref+.4];

figure('Position',[0,0, aspectRatioN]);
hYawRate = plot(wB.t-t0,wB.r,'k');
hold on
hYawRef  = plot(wB.t-t0,wB.r_ref(:,1),'--r');
hRiseTime1 = plot([xlim(1) riseTime1],0.9*[ref ref],'--k');
hRiseTime2 = plot([riseTime1 riseTime1],[ylim(1) .9*ref],'--k');

set(gca,'XLim',xlim,'YLim',ylim)
set(get(gca,'Children'),'LineWidth',LW)
set(hRiseTime1,'LineWidth',1)
set(hRiseTime2,'LineWidth',1)
set(gca,'YTick',[0 ref*0.9 ref])
set(gca,'Xtick',[xlim(1), 0, riseTime1,xlim(2)])
xlabel('Time [s]')
ylabel('Yaw Rate [rad/s]')

legend('Yaw rate','Yaw rate reference','Location','NorthWest')

strSpeed = sprintf('Speed = %i km/h',round(mean(wB.u(yawRate90-200))*3.6));
text(xlim(1)+0.05*(xlim(2)-xlim(1)),ylim(1)+0.75*(ylim(2)-ylim(1)),strSpeed,'FontSize',FS)

set(gca,'FontSize',FS)



%% Plot r and rref with B = 0
idx = find(woB.r_ref(:,1) > 0);
t0  = woB.t(idx(1));

[~, yawRate90] = find(woB.r > 0.9*max(woB.r_ref(:,1)));
interp1(woB.t,woB.r,woB.r(yawRate90(1)));
riseTime2 = interp1(woB.r(yawRate90(1)-1:yawRate90(1)),woB.t(yawRate90(1)-1:yawRate90(1)),0.9*ref)-t0;

figure('Position',[0,0, aspectRatioN]);
hYawRate = plot(woB.t-t0,woB.r,'k');
hold on
hYawRef  = plot(woB.t-t0,woB.r_ref(:,1),'--r');
hRiseTime1 = plot([xlim(1) riseTime2],[ref ref]*0.9,'--k');
hRiseTime2 = plot([riseTime2 riseTime2],[ylim(1) 0.9*ref],'--k');

set(gca,'XLim',xlim,'YLim',ylim)
set(get(gca,'Children'),'LineWidth',LW)
set(hRiseTime1,'LineWidth',1)
set(hRiseTime2,'LineWidth',1)
set(gca,'YTick',[0 ref*0.9 ref])
set(gca,'Xtick',[xlim(1), 0, riseTime2,xlim(2)])

legend('Yaw rate','Yaw rate reference','Location','NorthWest')
text(xlim(1)+0.05*(xlim(2)-xlim(1)),ylim(1)+0.75*(ylim(2)-ylim(1)),strSpeed,'FontSize',FS)

xlabel('Time [s]')
ylabel('Yaw Rate [rad/s]')
set(gca,'FontSize',FS)

%% Calculate overshoot

OsWB = max(wB.r(1,:)) - 1;
OsWoB = max(woB.r(1,:)) - 1;




