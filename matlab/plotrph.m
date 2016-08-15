function f = plotrph(data,ground)

if any(strcmp('att',fieldnames(ground)))
    ground.gyro_attitude = [ground.att(3,:);ground.att(2,:);ground.att(1,:)]'*180/pi;
end
samp = data.att(2:end,:);


[~, st] = min(abs(ground.t-data.t(1)));
[~, nd] = min(abs(ground.t-data.t(end)));



ground.gyro_attitude(:,1) = resample2(ground.t(st:nd),ground.gyro_attitude(st:nd,1),data.t');
ground.gyro_attitude(:,2) = resample2(ground.t(st:nd),ground.gyro_attitude(st:nd,2),data.t');
ground.gyro_attitude(:,3) = resample2(ground.t(st:nd),ground.gyro_attitude(st:nd,3),data.t');

ground.gyro_attitude = ground.gyro_attitude(2:end,:);

figure;
hold on;

ground.t = ground.t-data.stamp(1);
data.t = data.t-data.t(1);

ground.gyro_attitude=wrapToPi(ground.gyro_attitude*pi/180)*180/pi;
samp = wrapToPi(samp)*180/pi;


subplot(3,1,1);
hold on;
data_t = taxis(data.t(2:end));

plot(data_t,ground.gyro_attitude(:,3),'-r');
plot(data_t,samp(:,1),'-b');
title('Roll');
ylabel('Degrees');
xlabel(tlabel(data.t));
xlim([data_t(1),data_t(end)]);
grid on;

subplot(3,1,2);
hold on;
plot(data_t,ground.gyro_attitude(:,2),'-r');
plot(data_t,samp(:,2),'-b');



title('Pitch');
ylabel('Degrees');
xlabel(tlabel(data.t));
xlim([data_t(1),data_t(end)]);
grid on;

subplot(3,1,3);
hold on;
plot(data_t,ground.gyro_attitude(:,1),'-r');
plot(data_t,samp(:,3),'-b');
title('Heading');
ylabel('Degrees');
xlabel(tlabel(data.t));
xlim([data_t(1),data_t(end)]);
grid on;
legend('Phins','KVH');







tmp_roll = ground.gyro_attitude(:,3);
gnd(:,3) = ground.gyro_attitude(:,1);
gnd(:,1) = tmp_roll;
gnd(:,2) = ground.gyro_attitude(:,2);

diff = unwrap(samp*pi/180)-unwrap(gnd*pi/180);
diff = wrapToPi(diff)*180/pi;

figure;
hold on;

subplot(3,1,1);
hold on;
plot(data_t,diff(:,1));
title('Roll');
ylabel('Degrees');
xlabel(tlabel(data.t));
xlim([data_t(1),data_t(end)]);
grid on;

subplot(3,1,2);
hold on;
plot(data_t,diff(:,2));
title('Pitch');
ylabel('Degrees');
xlabel(tlabel(data.t));
xlim([data_t(1),data_t(end)]);
grid on;

subplot(3,1,3);
hold on;
plot(data_t,diff(:,3));
title('Heading');
ylabel('Degrees');
xlabel(tlabel(data.t));
xlim([data_t(1),data_t(end)]);
grid on;




