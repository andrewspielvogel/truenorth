function out = int_plant(samp)

num_samp = size(samp.acc,2);

t = samp.stamp-samp.stamp(1);

b_ang = zeros(3,num_samp);
acc   = zeros(3,num_samp);
b_acc = zeros(3,num_samp);
b_z   = zeros(3,num_samp);

acc(:,1) = samp.acc(:,1);




k1 = 1;
k2 = .0001;
k3 = .0001;
k4 = .000001;


for i=2:num_samp
    
    da = acc(:,i-1) - samp.acc(:,i-1);
    dt = t(i)-t(i-1);
    
    da_est = skew(-samp.ang(:,i-1))*acc(:,i-1) + skew(samp.ang(:,i-1))*b_acc(:,i-1) + skew(samp.acc(:,i-1))*b_ang(:,i-1) - b_z(:,i-1) - k1*da;
    dab    =  k2*skew(samp.ang(:,i-1))*da;
    dwb    = -k3*skew(samp.acc(:,i-1))*da;
    dzb    =  k4*da;
    
    acc(:,i)   = acc(:,i-1)   + dt*da_est;
    b_acc(:,i) = b_acc(:,i-1) + dt*dab;
    b_ang(:,i) = b_ang(:,i-1) + dt*dwb;
    b_z(:,i)   = b_z(:,i-1)   + dt*dzb;
    
end


out.acc = acc;
out.bias.acc = b_acc;
out.bias.ang = b_ang;
out.bias.z = b_z;
out.t = t;
out.hz = samp.hz;
out.true = samp.true;