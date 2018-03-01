function out = plot_att_error2(data,data2,data3,hz)
set(gcf,'renderer','Painters')

phins = resample2(data(1:hz/10:end-hz/10,1),data(hz/10:hz/10:end,5:7),data(:,1),'linear');
t = data(:,1) - data(1,1);

subplot(3,2,1);
hold on;
title("Roll");
ylabel("Degrees");
plot(t,data(:,2)*180/pi,t,data2(:,2)*180/pi,t,data3(:,2)*180/pi,t,phins(:,1)*180/pi);grid on;xlim([t(1),t(end)]);
subplot(3,2,3);
hold on;
title("Pitch");
ylabel("Degrees");
plot(t,data(:,3)*180/pi,t,data2(:,3)*180/pi,t,data3(:,3)*180/pi,t,phins(:,2)*180/pi);grid on;xlim([t(1),t(end)]);
subplot(3,2,5);
hold on;
title("Heading");
ylabel("Degrees");
xlabel("Time [s]");
plot(t,data(:,4)*180/pi,t,data2(:,4)*180/pi,t,data3(:,4)*180/pi,t,phins(:,3)*180/pi);grid on;xlim([t(1),t(end)]);
legend("NOISE","NO\_NOISE","FILT","True",'Location','south','Orientation','horizontal');
%figure;plot(t,data(:,4)*180/pi,t,phins(:,3)*180/pi);grid on;
subplot(3,2,2);
hold on;
title("Roll Error");
ylabel("Degrees");
plot(t,data(:,2)*180/pi-phins(:,1)*180/pi,t,data2(:,2)*180/pi-phins(:,1)*180/pi,t,data3(:,2)*180/pi-phins(:,1)*180/pi);grid on;ylim([-2,2]);xlim([t(1),t(end)]);
subplot(3,2,4);
hold on;
title("Pitch Error");
ylabel("Degrees");
plot(t,data(:,3)*180/pi-phins(:,2)*180/pi,t,data2(:,3)*180/pi-phins(:,2)*180/pi,t,data3(:,3)*180/pi-phins(:,2)*180/pi);grid on;ylim([-2,2]);xlim([t(1),t(end)]);
subplot(3,2,6);
hold on;
title("Heading Error");
ylabel("Degrees");
xlabel("Time [s]");
plot(t,data(:,4)*180/pi-phins(:,3)*180/pi,t,data2(:,4)*180/pi-phins(:,3)*180/pi,t,data3(:,4)*180/pi-phins(:,3)*180/pi);grid on;ylim([-25,25]);xlim([t(1),t(end)]);
%legend("NOISE","NO\_NOISE","FILT",'Location','south','Orientation','horizontal');
%figure;plot(data(:,1),dot(data(:,17:19),data(:,17:19),2)/2);grid on;title('V(t)');xlabel('Time [s]');ylabel('V(t)');
%figure;hold on;plot(data(:,1),dot(data(:,17:19),data(:,8:10),2));grid on;
%plot(data(:,1),dot(data(:,17:19),data(:,11:13),2));grid on;
%plot(data(:,1),dot(data(:,17:19),(data(:,8:10)+data(:,11:13)),2));grid on;legend({'$\dot{V}_{a_g}(t)$','$\dot{V}_e(t)$','$\dot{V}(t)$'},'Interpreter','latex');xlabel('Time [s]');
