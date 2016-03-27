function f = plot_adap(out)

figure
subplot(3,1,1);
plot(out.t,out.bias.acc(:,1),out.t,repmat(out.true.bias.acc(1),1,size(out.t,1)));
xlabel('time [s]');
ylabel('acc_x [g]');

subplot(3,1,2);
plot(out.t,out.bias.acc(:,2),out.t,repmat(out.true.bias.acc(2),1,size(out.t,1)));
xlabel('time [s]');
ylabel('acc_y [g]');

subplot(3,1,3);
plot(out.t,out.bias.acc(:,3),out.t,repmat(out.true.bias.acc(3),1,size(out.t,1)));
xlabel('time [s]');
ylabel('acc_z [g]');

figure
subplot(3,1,1);
plot(out.t,out.bias.ang(:,1),out.t,repmat(out.true.bias.ang(1),1,size(out.t,1)));
xlabel('time [s]');
ylabel('ang_x [rad/s]');

subplot(3,1,2);
plot(out.t,out.bias.ang(:,2),out.t,repmat(out.true.bias.ang(2),1,size(out.t,1)));
xlabel('time [s]');
ylabel('ang_y [rad/s]');

subplot(3,1,3);
plot(out.t,out.bias.ang(:,3),out.t,repmat(out.true.bias.ang(3),1,size(out.t,1)));
xlabel('time [s]');
ylabel('ang_z [rad/s]');

legend('est','true');