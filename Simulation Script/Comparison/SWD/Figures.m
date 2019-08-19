clear all
close all
wB = load('SWDWithB.mat');
woB = load('SWDWoB.mat');

LW = 3;
FS = 20;
FN = 'Helvetica';
aspectRatioW = [16 10]*70;
aspectRatioN = [8 10]*70;

%% Plot yaw rate and reference with Beta = 1

xlim = [-.2 3];
ylim = [-1.1 1.1];
t0 = 10;

f1=figure('Position',[0,0, aspectRatioN],'units','pixel');
hYawRate = plot(wB.t-t0,wB.r,'k');
hold on
hYawRef  = plot(wB.t-t0,wB.r_ref(:,1),'--r');

set(gca,'XLim',xlim,'YLim',ylim)
set(get(gca,'Children'),'LineWidth',LW)
xlabel('Time [s]')
ylabel('Yaw Rate [rad/s]')

legend('Yaw rate','Yaw rate reference')

strSpeed = sprintf('Entry speed = %i km/h',round(max(wB.u)*3.6));
text(xlim(1)+0.52*(xlim(2)-xlim(1)),ylim(1)+0.85*(ylim(2)-ylim(1)),strSpeed,'FontSize',FS,'FontName',FN)

set(gca,'FontSize',FS)
set(gca,'FontName',FN)
set(gca,'LineWidth',LW)


%% Plot r and rref with B = 0

f2=figure('Position',[0,0, aspectRatioN]);
hYawRate = plot(woB.t-t0,woB.r,'k');
hold on
hYawRef  = plot(woB.t-t0,woB.r_ref(:,1),'--r');

set(gca,'XLim',xlim,'YLim',ylim)
set(get(gca,'Children'),'LineWidth',LW)
xlabel('Time [s]')
ylabel('Yaw Rate [rad/s]')

legend('Yaw rate','Yaw rate reference')

text(xlim(1)+0.52*(xlim(2)-xlim(1)),ylim(1)+0.85*(ylim(2)-ylim(1)),strSpeed,'FontSize',FS,'FontName',FN);

set(gca,'FontSize',FS)
set(gca,'FontName',FN)
set(gca,'LineWidth',LW)

%%

[~, idx1] = min(abs((wB.t-t0)-1.63));
t = wB.t(idx1:end)-wB.t(idx1);

idx12 = find(wB.r(idx1:end)>-0.1);
idx12 = idx12(1);
riseTime1 = interp1(wB.r(idx1+idx12-2:idx1+idx12-1),t(idx12-1:idx12),-0.1);

idx22 = find(woB.r(idx1:end)>-0.1);
idx22 = idx22(1);
riseTime2 = interp1(woB.r(idx1+idx22-2:idx1+idx22-1),t(idx22-1:idx22),-0.1);
 
% riseTime1 = wB.t(idx1+idx12(1))-wB.t(idx1);
% 
% idx22 = find(woB.r(idx1:end)>-0.1);
% riseTime2 = wB.t(idx1+idx22(1))-wB.t(idx1);

%%
beta1 = atan2(wB.v,wB.u);
beta2 = atan2(woB.v,woB.u);

figure('Position',[0,0, aspectRatioW],'units','pixel')

set(gca,'FontSize',FS)
set(gca,'FontName',FN)
set(gca,'LineWidth',LW)

hBeta1 = plot(wB.t-t0,beta1*180/pi,'ro-');
hold on
hBeta2 = plot(wB.t-t0,beta2*180/pi,'kv-');

set(gca,'xlim',xlim,'LineWidth',LW,'FontSize',FS)
set(hBeta1,'MarkerSize',10,'LineWidth',LW,'MarkerIndices',(1:20:length(beta1)))
set(hBeta2,'MarkerSize',10,'LineWidth',LW,'MarkerIndices',(10:20:length(beta1)))

xlabel('Time [s]')
ylabel('Body side slip [degree]')
legend('With Yaw-Beta controller','With Yaw controller')


