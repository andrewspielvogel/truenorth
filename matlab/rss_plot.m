hz = 100;
figure;
subplot(6,2,2);
plot(data1(:,1)-data1(1,1),data1(:,2)*180/pi-data1(:,5)*180/pi,data3(:,1)-data3(1,1),data3(:,2)*180/pi-data3(:,5)*180/pi,data4(:,1)-data4(1,1),data4(:,2)*180/pi-data4(:,5)*180/pi);grid on;
ylabel("Degrees");
ylim([-1,1]);
xlim([0,size(data3(:,1),1)/hz]);
title("Roll Error");

subplot(6,2,4);
plot(data1(:,1)-data1(1,1),data1(:,3)*180/pi-data1(:,6)*180/pi,data3(:,1)-data3(1,1),data3(:,3)*180/pi-data3(:,6)*180/pi,data4(:,1)-data4(1,1),data4(:,3)*180/pi-data4(:,6)*180/pi);grid on;
ylabel("Degrees");
ylim([-1,1]);
xlim([0,size(data3(:,1),1)/hz]);
title("Pitch Error");

subplot(6,2,6);
plot(data1(:,1)-data1(1,1),data1(:,4)*180/pi-data1(:,7)*180/pi,data3(:,1)-data3(1,1),data3(:,4)*180/pi-data3(:,7)*180/pi,data4(:,1)-data4(1,1),data4(:,4)*180/pi-data4(:,7)*180/pi);grid on;
ylabel("Degrees");
ylim([-20,20]);
xlim([0,size(data3(:,1),1)/hz]);
title("Heading Error");


data1w = data1(:,8:10) - repmat(bias.ang',[size(data1(:,1),1),1]);
data3w = data3(:,8:10) - repmat(bias.ang',[size(data3(:,1),1),1]);
data4w = data4(:,8:10) - repmat(bias.ang',[size(data4(:,1),1),1]);


subplot(6,2,8);
plot(data1(:,1)-data1(1,1),data1w(:,1),data3(:,1)-data3(1,1),data3w(:,1),data4(:,1)-data4(1,1),data4w(:,1));grid on;
title("X Axis Gyro Bias Error");
ylim([-0.00005,0.00005]);
xlim([0,size(data3(:,1),1)/hz]);
ylabel("rad/s");

subplot(6,2,10);
plot(data1(:,1)-data1(1,1),data1w(:,2),data3(:,1)-data3(1,1),data3w(:,2),data4(:,1)-data4(1,1),data4w(:,2));grid on;
title("Y Axis Gyro Bias Error");
ylim([-0.00005,0.00005]);
xlim([0,size(data3(:,1),1)/hz]);
ylabel("rad/s");

subplot(6,2,12);
plot(data1(:,1)-data1(1,1),data1w(:,3),data3(:,1)-data3(1,1),data3w(:,3),data4(:,1)-data4(1,1),data4w(:,3));grid on;
title("Z Axis Gyro Bias Error");
ylim([-0.00005,0.00005]);
xlim([0,size(data3(:,1),1)/hz]);
ylabel("rad/s");
xlabel("Seconds");


data1a = data1(:,14:16) - repmat(bias.acc'*9.81,[size(data1(:,1),1),1]);
data3a = data3(:,14:16) - repmat(bias.acc'*9.81,[size(data3(:,1),1),1]);
data4a = data4(:,14:16) - repmat(bias.acc'*9.81,[size(data4(:,1),1),1]);


subplot(6,2,7);
plot(data1(:,1)-data1(1,1),data1a(:,1),data3(:,1)-data3(1,1),data3a(:,1),data4(:,1)-data4(1,1),data4a(:,1));grid on;
title("X Axis Accelerometer Bias Error");
ylim([-0.05,0.05]);
xlim([0,size(data3(:,1),1)/hz]);
ylabel("m/s^2");

subplot(6,2,9);
plot(data1(:,1)-data1(1,1),data1a(:,2),data3(:,1)-data3(1,1),data3a(:,2),data4(:,1)-data4(1,1),data4a(:,2));grid on;
title("Y Axis Acclerometer Bias Error");
ylim([-0.05,0.05]);
xlim([0,size(data3(:,1),1)/hz]);
ylabel("m/s^2");

subplot(6,2,11);
plot(data1(:,1)-data1(1,1),data1a(:,3),data3(:,1)-data3(1,1),data3a(:,3),data4(:,1)-data4(1,1),data4a(:,3));grid on;
title("Z Axis Acclerometer Bias Error");
ylim([-0.05,0.05]);
xlim([0,size(data3(:,1),1)/hz]);
ylabel("m/s^2");
xlabel("Seconds");



subplot(6,2,1);
plot(data1(:,1)-data1(1,1),data1(:,2)*180/pi,data3(:,1)-data3(1,1),data3(:,2)*180/pi,data4(:,1)-data4(1,1),data4(:,2)*180/pi);grid on;
ylabel("Degrees");
ylim([-1,1]);
xlim([0,size(data3(:,1),1)/hz]);
title("Estimated Roll");

subplot(6,2,3);
plot(data1(:,1)-data1(1,1),data1(:,3)*180/pi,data3(:,1)-data3(1,1),data3(:,3)*180/pi,data4(:,1)-data4(1,1),data4(:,3)*180/pi);grid on;
ylabel("Degrees");
ylim([-1,1]);
xlim([0,size(data3(:,1),1)/hz]);
title("Estimated Pitch");

subplot(6,2,5);
plot(data1(:,1)-data1(1,1),data1(:,4)*180/pi,data3(:,1)-data3(1,1),data3(:,4)*180/pi,data4(:,1)-data4(1,1),data4(:,4)*180/pi);grid on;
ylabel("Degrees");
xlim([0,size(data3(:,1),1)/hz]);
title("Estimated Heading");

legend("exp1","exp2","exp3","orientation","horizontal");
