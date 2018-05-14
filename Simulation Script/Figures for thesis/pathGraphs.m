clear all
close all

LW = 3;
FS = 20;
aspectRatio = [16 10]*70;
plotPath = true;

x= (0:0.5:10)';
y= sin(x*2*pi/10)+(rand(size(x))-0.5)*.15;
localPath = [x,y];
curveEstimation;
dx =1.5;


if plotPath
    figure('Position',[0.5, 0.5 , aspectRatio]);
    plot(x,y,'*k')
    hold on
    plot(x,X*C,'k--')
    plot([x(7), x(7)+dx+1],[y(7), (y(7)+yDot(7)*(dx+1))])
    plot([x(7), x(7)+dx+.1],[y(7) y(7)],'k','LineWidth',1)
    plot([x(7)+dx, x(7)+dx],[y(7)+.1 y(7)+yDot(7)*dx],'k','LineWidth',1)
    axis equal
    
    xlabel('y [m]')
    ylabel('x [m]')
    legend('Local path','Estimated path')
    
    set(gca,'FontSize',FS)
end
