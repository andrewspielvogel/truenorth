function out = plot_components(t,data,x_label_,y_label_,title_)

figure;

subplot(3,1,1);
plot(t,data(:,1));grid on;title(title_);ylabel(y_label_);xlim([0,t(end)]);

subplot(3,1,2);
plot(t,data(:,2));grid on;ylabel(y_label_);xlim([0,t(end)]);

subplot(3,1,3);
plot(t,data(:,3));grid on;ylabel(y_label_);xlabel(x_label_);xlim([0,t(end)]);