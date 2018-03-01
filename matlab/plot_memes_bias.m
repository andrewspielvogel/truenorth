function out = plot_memes_bias(samp,data,bias)

declination = 11 + 11/60;
declination = declination*pi/180;

% skip = 100;
% samp.t = samp.t(1:skip:end);
% samp.att = samp.att(:,1:skip:end);
% samp.mag = samp.mag(:,1:skip:end);
% samp.acc = samp.acc(:,1:skip:end);
% data = data(1:skip:end,:);

figure(1);
subplot(3,1,1);hold on;

plot(data(:,1),data(:,8));grid on;ylabel('g');title('a_b');xlim([0,data(end,1)]);ylim([-0.005,0.005]);
subplot(3,1,2);hold on;

plot(data(:,1),data(:,9));grid on;ylabel('g');xlim([0,data(end,1)]);ylim([-0.005,0.005]);
subplot(3,1,3);hold on;

plot(data(:,1),data(:,10));grid on;ylabel('g');xlabel('Seconds');xlim([0,data(end,1)]);ylim([-0.01,0.01]);

figure(2);
subplot(3,1,1);hold on;

plot(data(:,1),data(:,11));grid on;ylabel('rad/s');title('w_b');xlim([0,data(end,1)]);
subplot(3,1,2);hold on;

plot(data(:,1),data(:,12));grid on;ylabel('rad/s');xlim([0,data(end,1)]);
subplot(3,1,3);hold on;

plot(data(:,1),data(:,13));grid on;ylabel('rad/s');xlabel('Seconds');xlim([0,data(end,1)]);

figure(3);
hold on;
subplot(3,1,1);hold on;

plot(data(:,1),data(:,14));grid on;ylabel('gauss');title('m_b');xlim([0,data(end,1)]);
subplot(3,1,2);hold on;

plot(data(:,1),data(:,15));grid on;ylabel('gauss');xlim([0,data(end,1)]);
subplot(3,1,3);hold on;

plot(data(:,1),data(:,16));grid on;ylabel('gauss');xlabel('Seconds');xlim([0,data(end,1)]);

figure(4);
hold on;
subplot(3,1,1);hold on;

plot(data(:,1),data(:,8) - bias.acc(1));grid on;title('a_b error');xlim([0,data(end,1)]);ylim([-0.002,0.002]);
subplot(3,1,2);hold on;

plot(data(:,1),data(:,9) - bias.acc(2));grid on;xlim([0,data(end,1)]);ylim([-0.002,0.002]);
subplot(3,1,3);hold on;

plot(data(:,1),data(:,10) - bias.acc(3));grid on;xlabel('Seconds');xlim([0,data(end,1)]);ylim([-0.01,0.01]);

figure(5);
hold on;
subplot(3,1,1);hold on;

plot(data(:,1),data(:,11) - bias.ang(1));grid on;title('w_b error');xlim([0,data(end,1)]);
subplot(3,1,2);hold on;

plot(data(:,1),data(:,12) - bias.ang(2));grid on;xlim([0,data(end,1)]);
subplot(3,1,3);hold on;

plot(data(:,1),data(:,13) - bias.ang(3));grid on;xlabel('Seconds');xlim([0,data(end,1)]);

figure(6);
hold on;
subplot(3,1,1);hold on;

plot(data(:,1),data(:,14) - bias.mag(1));grid on;title('m_b error');xlim([0,data(end,1)]);ylim([-0.05,0.05]);
subplot(3,1,2);hold on;

plot(data(:,1),data(:,15) - bias.mag(2));grid on;xlim([0,data(end,1)]);ylim([-0.05,0.05]);
subplot(3,1,3);hold on;

plot(data(:,1),data(:,16) - bias.mag(3));grid on;xlabel('Seconds');xlim([0,data(end,1)]);ylim([-0.05,0.05]);

figure(7);hold on;plot(samp.t,unwrap(samp.att'-data(:,17:19) + repmat([0;0;declination],[1,size(data,1)])')*180/pi);grid on;xlabel('Seconds');ylabel('Degrees');title('Attitude Error');xlim([0,data(end,1)]);

figure(8);
hold on;
subplot(3,1,1);hold on;

plot(samp.t,samp.att(1,:)*180/pi,data(:,1),data(:,17)*180/pi);grid on;xlabel('Seconds');ylabel('Roll (Degrees)');xlim([0,data(end,1)]);

subplot(3,1,2);hold on;

plot(samp.t,samp.att(2,:)*180/pi,data(:,1),data(:,18)*180/pi);grid on;xlabel('Seconds');ylabel('Pitch (Degrees)');xlim([0,data(end,1)]);

subplot(3,1,3);hold on;
plot(samp.t,samp.att(3,:)*180/pi,data(:,1),data(:,19)*180/pi);grid on;xlabel('Seconds');ylabel('Pitch (Degrees)');xlim([0,data(end,1)]);


