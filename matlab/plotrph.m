function f = plotrph(data,ground,align)


samp(:,1) = data.att(:,1)-align(1);
samp(:,2) = data.att(:,2)-align(2);
samp(:,3) = data.att(:,3)-align(3);

figure;
hold on;


ground.t = ground.t-data.t(1);
data.t = data.t-data.t(1);

ground.gyro_attitude=wrapToPi(ground.gyro_attitude*pi/180)*180/pi;
samp = wrapToPi(samp)*180/pi;

subplot(3,1,1);
hold on;
plot(ground.t,ground.gyro_attitude(:,3),'-r');
plot(data.t,samp(:,1),'-b');
title('Roll');
ylabel('Degrees');
xlabel('Time [s]');
xlim([data.t(1),data.t(end)]);
grid on;

subplot(3,1,2);
hold on;
plot(ground.t,ground.gyro_attitude(:,2),'-r');
plot(data.t,samp(:,2),'-b');



title('Pitch');
ylabel('Degrees');
xlabel('Time [s]');
xlim([data.t(1),data.t(end)]);
grid on;

subplot(3,1,3);
hold on;
plot(ground.t,ground.gyro_attitude(:,1),'-r');
plot(data.t,samp(:,3),'-b');
title('Heading');
ylabel('Degrees');
xlabel('Time [s]');
xlim([data.t(1),data.t(end)]);
grid on;
legend('Phins','KVH');



[~, st] = min(abs(ground.t-data.t(1)));
[~, nd] = min(abs(ground.t-data.t(end)));



gnd(:,1) = resample2(ground.t(st:nd),ground.gyro_attitude(st:nd,1),data.t');
gnd(:,2) = resample2(ground.t(st:nd),ground.gyro_attitude(st:nd,2),data.t');
gnd(:,3) = resample2(ground.t(st:nd),ground.gyro_attitude(st:nd,3),data.t');

tmp_roll = gnd(:,3);
gnd(:,3) = gnd(:,1);
gnd(:,1) = tmp_roll;

diff = samp-gnd;

figure;
hold on;

subplot(3,1,1);
hold on;
plot(data.t,diff(:,1));
title('Roll');
ylabel('Degrees');
xlabel('Time [s]');
xlim([data.t(1),data.t(end)]);
grid on;

subplot(3,1,2);
hold on;
plot(data.t,diff(:,2));
title('Pitch');
ylabel('Degrees');
xlabel('Time [s]');
xlim([data.t(1),data.t(end)]);
grid on;

subplot(3,1,3);
hold on;
plot(data.t,diff(:,3));
title('Heading');
ylabel('Degrees');
xlabel('Time [s]');
xlim([data.t(1),data.t(end)]);
grid on;




