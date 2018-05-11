clear all
close all
load recordings.mat

LW = 3;
FS = 20;
aspectRatio = [16 10]*70;
plotTorque = false;
plotMaxTorque =true;

if plotTorque
    figure('Position',[0.5, 0.5 , aspectRatio]);
    
    span = (650:780);
    hMotorTorque = plot(signals.time(span)-signals.time(span(1)),signals.pe_motorTorque_RL(span));
    hold on
    hMotorTorqueSet = plot(signals.time(span)-signals.time(span(1)),signals.rn_motorTorqueSet_RL(span),'--');
    
    set(gca,'XLim',[signals.time([span(1) span(end)])-signals.time(span(1))])
    set(hMotorTorque,'LineWidth',LW,'color','k')
    set(hMotorTorqueSet,'LineWidth',LW,'color','k')
    
    
    xlabel('Time [s]')
    ylabel('Torque [Nm]')
    legend('Motor torque','Torque request')
    
    set(gca,'FontSize',FS)
end

if plotMaxTorque
    figure('Position',[0.5, 0.5 , aspectRatio]);
    omega = 0:0.01:20e3*pi/30;
    Torque = omega;
    Torque(omega<1.5833e+03) = 24;
    Torque(omega>=10000*pi/30) = 2*24*(1-omega(omega>=10000*pi/30)/2e4/pi*30);
    Torque(end) = 0;
    
    hMaxMotorTorque = plot(omega*30/pi,Torque);
    hold on
    plot([1e4 1e4],[0 24],'--k','LineWidth',2);
    set(hMaxMotorTorque,'LineWidth',LW,'color','k')
    set(gca,'Xtick',[0 5000 10000 15000 20000])
    set(gca,'XTickLabel',{'0','5000','10000','15000','20000'})
    
    
    xlabel('Motor speed [rpm]')
    ylabel('Torque [Nm]')
    
%    set(gca,'FontSize',FS)
end