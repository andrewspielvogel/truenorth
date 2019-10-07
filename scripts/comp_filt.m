function out = comp_filt(samp)

num_samp = size(samp.acc,1);

a_hat = zeros(3,num_samp);
alpha = .3;

a_hat(:,1) = samp.acc(1,:)';

for i=2:num_samp
    
    dt = samp.t(i) - samp.t(i-1);
    a_hat(:,i) = (1-alpha)*(a_hat(:,i-1) - skew(samp.ang(i,:))*samp.acc(i,:)'*dt) + alpha*samp.acc(i,:)';
    
end

out.a_hat = a_hat;
out.t = samp.t;
figure;plot(samp.t,samp.acc,out.t,out.a_hat);grid on;