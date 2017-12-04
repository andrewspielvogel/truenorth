function out = plot_att_error(data,hz,save_figs)


phins = resample2(data(1:hz/10:end-hz/10,1),data(hz/10:hz/10:end,5:7),data(:,1),'linear');
wEn = data(:,17:19);%resample2(data(1:hz/10:end-hz/10,1),data(hz/10:hz/10:end,17:19),data(:,1),'linear');
t = data(:,1) - data(1,1);
figure;
subplot(3,2,1);
hold on;
title("Roll");
ylabel("Degrees");
plot(t,data(:,2)*180/pi,t,phins(:,1)*180/pi);grid on;xlim([t(1),t(end)]);
subplot(3,2,3);
hold on;
title("Pitch");
ylabel("Degrees");
plot(t,data(:,3)*180/pi,t,phins(:,2)*180/pi);grid on;xlim([t(1),t(end)]);
subplot(3,2,5);
hold on;
title("Heading");
ylabel("Degrees");
xlabel("Time [s]");
plot(t,data(:,4)*180/pi,t,phins(:,3)*180/pi);grid on;xlim([t(1),t(end)]);
legend("KVH","PHINS",'Location','south','Orientation','horizontal');
%figure;plot(t,data(:,4)*180/pi,t,phins(:,3)*180/pi);grid on;
subplot(3,2,2);
hold on;
title("Roll Error");
ylabel("Degrees");  
plot(t,data(:,2)*180/pi-phins(:,1)*180/pi);grid on;ylim([-2,2]);xlim([t(1),t(end)]);
subplot(3,2,4);
hold on;
title("Pitch Error");
ylabel("Degrees");
plot(t,data(:,3)*180/pi-phins(:,2)*180/pi);grid on;ylim([-2,2]);xlim([t(1),t(end)]);
subplot(3,2,6);
hold on;
title("Heading Error");
ylabel("Degrees");
xlabel("Time [s]");
plot(t,data(:,4)*180/pi-phins(:,3)*180/pi);grid on;xlim([t(1),t(end)]);
if save_figs
    print('truenorth_notes/errors','-depsc');
end


figure;plot(t,data(:,4)*180/pi-phins(:,3)*180/pi);grid on;title('Heading Error');xlabel('Time [s]');ylabel('Heading Error');
figure;plot(t,data(:,11:13)-wEn);grid on;title('w_E_n Error');xlabel('Time [s]');ylabel('w_E_n Error');legend('x','y','z');
if save_figs
    print('truenorth_notes/w_E_n_errors','-depsc');
end

figure;plot(t,data(:,8:10));grid on;title('Gyro Bias');xlabel('Time [s]');legend('x','y','z');
if save_figs
    print('truenorth_notes/w_b','-depsc');
end

figure;plot(t,data(:,11:13),t,wEn);grid on;legend('x','y','z');
figure;plot(t,data(:,14:16));grid on;title('Acc Bias');legend('x','y','z');
if save_figs
    print('truenorth_notes/a_b','-depsc');
end 
figure;plot(t,data(:,17:19)*180/pi);grid on;
