function f = plotrph(data,ground)

data_rph = rph(data.R_in)*180/pi;

figure;
hold on;

% data.t = data.t-data.t(1);
% ground.t = ground.t - data.t(1);

subplot(3,1,1);
hold on;
%plot(ground.t,ground.gyro_attitude(:,3)-360,'-r');
plot(data.t,-data_rph(:,1),'-b');
title('Roll');
ylabel('Degrees');
xlabel('Time [s]');
xlim([data.t(1),data.t(end)]);
grid on;

subplot(3,1,2);
hold on;
%plot(ground.t,ground.gyro_attitude(:,2),'-r');
plot(data.t,data_rph(:,2),'-b');



title('Pitch');
ylabel('Degrees');
xlabel('Time [s]');
xlim([data.t(1),data.t(end)]);
grid on;

subplot(3,1,3);
hold on;
%plot(ground.t,ground.gyro_attitude(:,1),'-r');
plot(data.t,data_rph(:,3),'-b');
title('Heading');
ylabel('Degrees');
xlabel('Time [s]');
xlim([data.t(1),data.t(end)]);
grid on;
%legend('Phins','KVH');


