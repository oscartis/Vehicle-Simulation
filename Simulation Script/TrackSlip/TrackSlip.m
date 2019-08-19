close all

FS = 20;
LW = 3;

A = load('Controller_with_beta.mat');
B = load('Controller_without_beta.mat');

beta1 = atan2(A.v,A.u);
beta2 = atan2(B.v,B.u);

idx1 = abs(beta1) > 2*std(beta1);
idx2 = abs(beta2) > 2*std(beta1);

markerIdx1 = find(abs(beta1)>2*std(beta1));
markerIdx2 = find(abs(beta2)>2*std(beta1));
% markerIdx1 = idx1.*(1:length(idx1));
% markerIdx1(markerIdx1==0) = [];
% markerIdx2 = idx2.*(1:length(idx2));
% markerIdx2(markerIdx2==0) = [];
figure('Position',[0,0,16,10]*70);

plot(A.t,beta1*180/pi,'or-')
hold on
plot(B.t,beta2*180/pi,'vk-')

lines = get(gca,'Children');
set(lines(2),'MarkerIndices',(1:200:length(beta1)),'MarkerSize',10,'LineWidth',LW)
set(lines(1),'MarkerIndices',(1:200:length(beta2)),'MarkerSize',10,'LineWidth',LW)

legend('With Yaw-Beta controller','With Yaw controller')
xlabel('Time [s]')
ylabel('Body side slip [degree]')
set(gca,'FontSize',FS)
set(gca,'XLim',[0,A.t(end)],'YLim',[-max(abs([beta1,beta2])), max(abs([beta1,beta2]))]*180/pi*1.1)



% 
% i = 1;
% k = 1;
% copyIndx = idx2;
% maxBeta1 = [];
% maxBeta2 = [];
% while i < length(idx1)
%     tmpIndx = find(copyIndx > 0);
%     copyIndx(tmpIndx(1):1:tmpIndx(1)+250) = [];
%     maxBeta1(k,:) = beta1(tmpIndx:1:tmpIndx(1)+250);
%     maxBeta2(k,:) = beta2(tmpIndx:1:tmpIndx(1)+250);
%     i = i+tmpIndx(1)+250;
%     k = k+1;
% end
% 
% tmpIndx = find(copyIndx > 0);
% maxBeta1(k,:) = [beta1(tmpIndx), zeros(1,251-length(tmpIndx))];
% maxBeta2(k,:) = [beta2(tmpIndx), zeros(1,251-length(tmpIndx))];
% 
% K = max(abs(maxBeta2)')./max(abs(maxBeta1)');
% percentk = (K-1)*100;
% 
% txt = cell(1,k);
% for i = 1:k
%     txt{i} = sprintf('%0.1f%%',percentk(i));
% end



% figure
% 
% plot(A.trackPath(:,1),A.trackPath(:,2),'k')
% hold on
% hA = plot(A.X,A.Y,'^');
% hB = plot(B.X,B.Y,'v');
% 
% set(hA,'Marker','none')
% set(hB,'Marker','none')
