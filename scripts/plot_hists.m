function out = plot_hists(samp,name)

acc = samp.acc(1:floor(end/3),:);
ang = samp.ang(1:floor(end/3),:);

acc_mean = mean(acc);
ang_mean = mean(ang);
acc_std = std(acc);
ang_std = std(ang);

figure;plot(acc);grid on;
figure;plot(ang);grid on;

figure;
subplot(3,1,1);
histogram(acc(:,1));grid on;title(['acc - ' name]);

subplot(3,1,2);
histogram(acc(:,2));grid on;title(sprintf('acc mean = [%f,%f,%f]',acc_mean(1),acc_mean(2),acc_mean(3)));

subplot(3,1,3);
histogram(acc(:,3));grid on;title(sprintf('acc std = [%f,%f,%f]',acc_std(1),acc_std(2),acc_std(3)));

figure;
subplot(3,1,1);
histogram(ang(:,1));grid on;title(['ang - ' name]);

subplot(3,1,2);
histogram(ang(:,2));grid on;title(sprintf('ang mean = [%f,%f,%f]',ang_mean(1),ang_mean(2),ang_mean(3)));

subplot(3,1,3);
histogram(ang(:,3));grid on;title(sprintf('ang std = [%f,%f,%f]',ang_std(1),ang_std(2),ang_std(3)));