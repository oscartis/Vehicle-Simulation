addpath('FSG_track_v10')
addpath('Sine_wave')
close all

% A = load('Controller_without_beta.mat');
% B = load('Controller_with_beta.mat');

figure
plot(r(1,:)*180/pi)
hold on
plot(r_ref(:,1)*180/pi)
% 
% figure
% plot(B.r(1,:)*180/pi,'*')
% hold on
% plot(B.r_ref(:,1)*180/pi)
% hold on
% plot(A.trackPath(:,1),A.trackPath(:,2))
% hold on
% legend('Without \Beta','With \Beta')

