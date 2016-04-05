function f = plot_adap(out)

figure
subplot(3,1,1);
plot(out.t,out.bias.acc(1,:),out.t,repmat(out.true.bias.acc(1),1,size(out.t,1)));
xlabel('time [s]');
ylabel('acc_x [g]');
grid on;

subplot(3,1,2);
plot(out.t,out.bias.acc(2,:),out.t,repmat(out.true.bias.acc(2),1,size(out.t,1)));
xlabel('time [s]');
ylabel('acc_y [g]');
grid on;

subplot(3,1,3);
plot(out.t,out.bias.acc(:,3),out.t,repmat(out.true.bias.acc(3),1,size(out.t,1)));
xlabel('time [s]');
ylabel('acc_z [g]');
grid on;

figure
subplot(3,1,1);
plot(out.t,out.bias.ang(:,1),out.t,repmat(out.true.bias.ang(1),1,size(out.t,1)));
xlabel('time [s]');
ylabel('ang_x [rad/s]');
grid on;

subplot(3,1,2);
plot(out.t,out.bias.ang(:,2),out.t,repmat(out.true.bias.ang(2),1,size(out.t,1)));
xlabel('time [s]');
ylabel('ang_y [rad/s]');
grid on;

subplot(3,1,3);
plot(out.t,out.bias.ang(:,3),out.t,repmat(out.true.bias.ang(3),1,size(out.t,1)));
xlabel('time [s]');
ylabel('ang_z [rad/s]');
grid on;

legend('est','true');