x = (0:0.1:200);
y1 = 0*x(1:100);
y2 = 5*(sin((x(101:1300)-10)*2*pi/40-pi/2)+1);
y3 = sqrt(25^2-(x(1301:1550)-130).^2)-25;
y4 = -sqrt((25^2)-(x(1551:1801)-180).^2)-50;
y5 = x(1802:end)*0-75;
y = [y1, y2,y3,y4,y5];

plot(x,y,'x')
axis equal
figure(1)

customTrack = [x;y]';
save('customTrack.mat','customTrack')

