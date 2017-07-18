function out = plot_att_error(data,hz)

phins = resample2(data(1:hz/10:end-hz/10,1),data(hz/10:hz/10:end,5:7),data(:,1),'linear');

figure;
subplot(3,1,1);
hold on;
title("Roll");
ylabel("Degrees");
plot(data(:,1),data(:,2)*180/pi,data(:,1),phins(:,1)*180/pi);grid on;
subplot(3,1,2);
hold on;
title("Pitch");
ylabel("Degrees");
plot(data(:,1),data(:,3)*180/pi,data(:,1),phins(:,2)*180/pi);grid on;
subplot(3,1,3);
hold on;
title("Heading");
ylabel("Degrees");
xlabel("Time [s]");
plot(data(:,1),data(:,4)*180/pi,data(:,1),phins(:,3)*180/pi);grid on;
legend("KVH","PHINS",'Location','south','Orientation','horizontal');

figure;
subplot(3,1,1);
hold on;
title("Roll Error");
ylabel("Degrees");
plot(data(:,1),data(:,2)*180/pi-phins(:,1)*180/pi);grid on;
subplot(3,1,2);
hold on;
title("Pitch Error");
ylabel("Degrees");
plot(data(:,1),data(:,3)*180/pi-phins(:,2)*180/pi);grid on;
subplot(3,1,3);
hold on;
title("Heading Error");
ylabel("Degrees");
xlabel("Time [s]");
plot(data(:,1),data(:,4)*180/pi-phins(:,3)*180/pi);grid on;
