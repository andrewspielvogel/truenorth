function out = adap_Rbar(samp)

data = samp;
data.acc = samp.acc';
data.ang = samp.ang';

Rd   = get_R_d(data,eye(3));

num_samp  = size(samp.stamp,2);
u = zeros(3,num_samp);
y1 = zeros(2,num_samp);
y2 = ones(1,num_samp);
y = [y1;-y2];

for i=1:num_samp
    
    u(:,i) = Rd{i}*samp.acc(:,i);
    
end

out = adap_so3(y,u,samp.stamp);