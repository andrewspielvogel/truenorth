function out = read_kvh_proc(input,phins)

csv = csvread(input,1,1);

out.t = csv(:,7);
out.att = csv(:,8:10);
out.bias.ang = csv(:,11:13);
out.bias.acc = csv(:,17:19);
out.east = csv(:,14:16);
out.acc_hat = csv(:,20:22);
out.acc = csv(:,23:25);
out.ang = csv(:,26:28);
out.mag = csv(:,29:31);
out.temp = csv(:,32);
out.seq_num = csv(:,33);

num = size(out.t,1);

out.east_proj = zeros(3,num);
out.east_rot = zeros(3,num);

out.roll = zeros(1,num);
out.pitch = zeros(1,num);

for i = 1:num
    
    acc = out.acc_hat(i,:)' - out.bias.acc(i,:)';
    acc = acc/norm(acc);
    
    out.roll(i) = atan2(-acc(1),acc(2));
    out.pitch(i) = atan2(-acc(3),sqrt(acc(1)*acc(1)+acc(2)*acc(2)));
    P = acc*acc';
    
    out.east_proj(:,i) = (eye(3)-P)*out.east(i,:)';
    out.east_rot(:,i) = rph2rot([out.pitch(i);0;out.roll(i)])*out.east_proj(:,i);
        
end
%out.heading = resample2(out.t,unwrap(atan2(-out.east_proj(3,:),out.east_proj(1,:)))'*180/pi,phins.t);
out.heading = resample2(out.t,unwrap(atan2(-out.east_rot(3,:),out.east_rot(1,:)))'*180/pi,phins.t);


figure;plot(taxis(phins.t),out.heading,taxis(phins.t),unwrap(phins.att(:,3)*pi/180)*180/pi);grid on;xlabel(tlabel(phins.t));ylabel('degrees');title('Heading');legend('kvh','phins');
figure;plot(taxis(phins.t),out.heading-unwrap(phins.att(:,3)*pi/180)*180/pi);grid on;xlabel(tlabel(phins.t));ylabel('degrees');title('Heading Error');
figure;plot(taxis(out.t),out.bias.acc);grid on;xlabel(tlabel(out.t));ylabel('m/s^2');title('Acc Bias');
figure;plot(taxis(out.t),out.bias.ang);grid on;xlabel(tlabel(out.t));ylabel('rad/s');title('Ang Bias');
figure;plot(taxis(out.t),out.roll*180/pi,taxis(phins.t),phins.att(:,1));grid on;xlabel(tlabel(out.t));ylabel('degrees');title('Roll');
figure;plot(taxis(out.t),out.pitch*180/pi,taxis(phins.t),phins.att(:,2));grid on;xlabel(tlabel(out.t));ylabel('degrees');title('Pitch');
figure;plot(taxis(out.t),out.acc_hat-out.acc);grid on;xlabel(tlabel(out.t));ylabel('m/s^2');title('Delta a');
figure;plot(taxis(out.t),out.east);xlabel(tlabel(out.t));title('e(t)');
figure;plot(taxis(out.t),out.acc);grid on;xlabel(tlabel(out.t));ylabel('m/s^2');title('acc');
figure;plot(taxis(out.t),out.acc,taxis(out.t),out.acc_hat);grid on;xlabel(tlabel(out.t));ylabel('m/s^2');title('acc vs acchat');legend('acc','acchat');
figure;plot(taxis(out.t),out.ang);grid on;xlabel(tlabel(out.t));ylabel('rad/s');title('ang');legend('x','y','z');
figure;plot(taxis(out.t),cross(out.ang,out.east));grid on;xlabel(tlabel(out.t));title('ang cross east');legend('x','y','z');

