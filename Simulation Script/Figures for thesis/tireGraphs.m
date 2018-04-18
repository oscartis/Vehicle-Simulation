%InitializeModel;

%% x-direction
lineColors = flipud(jet(length(tireLoad)));
figure('Position',[0, 0, 4, 5]*200);
set(gca,'ColorOrder',lineColors,'NextPlot', 'replacechildren');
hLines = plot(tireSlipX*100,tireForceX);

lgdStr = cell(size(tireLoad));
for i = 1:length(tireLoad)
    lgdStr{i} = sprintf('%d N',round(tireLoad(i)));
end

set(gca,'LineWidth',3);
set(gca,'FontSize',20);
set(hLines,'LineWidth',3);

hlegend = legend(lgdStr,'Location','NorthWest','FontSize',14);
hlegTitle = get(hlegend,'Title');
set(hlegTitle,'String','Vertical Load');
xlabel('Tire slip [%]')
ylabel('F_x [N]')

%% y-direction

figure('Position',[0.5, 0.5 , 4, 5]*200);
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