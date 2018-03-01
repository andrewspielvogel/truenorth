function out = calc_wearth(samp,dt)

num = size(samp.t,2);

acc = zeros(3,num);
w_E = zeros(3,num);
east = zeros(3,num);
error = zeros(1,num);
acc(:,1) = [0;0;-1];

k_a=1;
k_E = .01;

for i=2:num
   
    da = (acc(:,i-1)-samp.acc(:,i))/dt;
    a_dot = -skew(samp.ang(:,i)-w_E(:,i-1))*samp.acc(:,i) - k_a*da;
    w_E_dot = -k_E*skew(samp.acc(:,i))*da - skew(samp.ang(:,i))*w_E(:,i-1);
    
    acc(:,i) = acc(:,i-1) + dt*a_dot;
    w_E(:,i) = w_E(:,i-1) + dt*w_E_dot;
    east(:,i) = skew(w_E(:,i))*samp.acc(:,i);
    east(:,i) = east(:,i)/norm(east(:,i));
    error(:,i) = asin(norm(cross(samp.E(:,i),east(:,i))))*180/pi;
    
end

out.acc = acc;
out.w_E = w_E;
out.t = samp.t;
out.E = east;
figure;plot(samp.t,acc,samp.t,samp.acc);grid on;
figure;plot(samp.t,w_E,samp.t,samp.w_E);grid on;
figure;plot(samp.t,w_E-samp.w_E);grid on;
figure;plot(samp.t,east,samp.t,samp.E);grid on;
figure;plot(samp.t,error);grid on;