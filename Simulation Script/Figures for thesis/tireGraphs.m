%InitializeModel;
addpath('/Users/oscar/github/Vehicle-Simulation/Simulation Script')
InitializeModel;

plotFx = false;
plotFy = true;
plotMu = true;
plotAy = false;

figureFactor = 150;
if plotFx || plotFy
    lineColors = flipud(jet(length(tireLoad)));
    lgdStr = cell(size(tireLoad));
    for i = 1:length(tireLoad)
        lgdStr{i} = sprintf('%d N',round(tireLoad(i)));
    end
end
tireLoad(end) = [];
%% x-direction
if plotFx
    figure('Position',[0, 0, 4, 5]*150);
    set(gca,'ColorOrder',lineColors,'NextPlot', 'replacechildren');
    hLines = plot(tireSlipX*100,tireForceX);
        
    set(gca,'LineWidth',3);
    set(gca,'FontSize',20);
    set(hLines,'LineWidth',3);
    
    hlegend = legend(lgdStr,'Location','NorthWest','FontSize',14);
    hlegTitle = get(hlegend,'Title');
    set(hlegTitle,'String','Vertical Load');
    xlabel('Tire slip [%]')
    ylabel('F_x [N]')
end
%% y-direction
if plotFy
    figure('Position',[0.5, 0.5 , 4, 5]*figureFactor);
    set(gca,'ColorOrder',lineColors,'NextPlot', 'replacechildren');
    hLines = plot(tireSlipY*180/pi,tireForceY);
    
    set(gca,'LineWidth',3);
    set(gca,'FontSize',20);
    set(hLines,'LineWidth',3);
    
    hlegend = legend(lgdStr,'Location','NorthWest','FontSize',14);
    hlegTitle = get(hlegend,'Title');
    set(hlegTitle,'String','Vertical Load');
    xlabel('Tire slip angle [°]')
    ylabel('F_y [N]')
end
%%
if plotMu
    figure('Position',[0.5, 0.5 , 4, 5]*figureFactor);
    maxForce = max(tireForceY(:,1:end-1));
    mu = maxForce./tireLoad;
    mu600 = interp1(tireLoad(1:end),mu,600);
    muFactor = 1.5/mu600;
  
    hmu1 = plot(tireLoad(1:length(mu)),mu,'-o');
    hold on
    hmu2 = plot(tireLoad(1:length(mu)),mu*muFactor,'--');
    h600N = plot([0, 600, 0;600, 600, 600],[mu600, mu600, 1.5;mu600, 0, 1.5],'--k');
    
    xlabel('Tire load [N]')
    ylabel('F_y normalized by F_z [-]')
    legend('Measured data','Estimated friction on test track')
    
    set(gca,'LineWidth',3);
    set(gca,'FontSize',20);
%     set(gca,'XTick',sort([tireLoad, 600]))
%     set(gca,'xticklabel',num2str(sort([tireLoad, 600]),'%1d'))
    set(hmu1,'LineWidth',3,'color','black');
    set(hmu1,'MarkerEdgeColor','red','MarkerSize',15)
    set(hmu2,'LineWidth',3,'color','black');
    set(hmu2,'MarkerEdgeColor','red','MarkerSize',15)
    set(gca,'XLim',[0 1600]);
end
%%
if plotAy
    figure('Position',[0.5, 0.5 , 4, 5]*figureFactor);
    
    hay = plot(time(1:8000),ay(1:8000),'-');
    
    xlabel('Time [s]')
    ylabel('Lateral acceleration [m/s^2]')
    
    set(gca,'LineWidth',3);
    set(gca,'FontSize',20);
    set(hay,'LineWidth',3,'color','black');
    set(hay,'MarkerEdgeColor','red','MarkerSize',15)
end