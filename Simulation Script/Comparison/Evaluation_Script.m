A = load('Controller_with_beta.mat');
B = load('Controller_without_beta.mat');

figure(1)
plot(A.x(3,:)./A.delta(1,:))
hold on
plot(B.x(3,:)./B.delta(1,:))
legend('WC','WoC')