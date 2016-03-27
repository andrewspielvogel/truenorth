function f = plot_adap(out)

figure
subplot(2,1,1);
plot(out.t,out.bias.acc,out.t,repmat(out.true.bias.acc,1,size(out.t,1)));
xlabel('time [s]');
ylabel('acc [g]');
subplot(2,1,2);
plot(out.t,out.bias.ang,out.t,repmat(out.true.bias.ang,1,size(out.t,1)));
xlabel('time [s]');
ylabel('ang [rad/s]');
legend('x_{est}','y_{est}','z_{est}','x_{true}','y_{true}','z_{true}');