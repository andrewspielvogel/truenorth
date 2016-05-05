function R = adap_so3(y,u,t)

num_samp = size(t,2);
R{num_samp} = eye(3);
R{1} = eye(3);
err = zeros(3,num_samp);

k=1;

for i=2:num_samp
    
    dt = t(i)-t(i-1);
    y_est = R{i-1}*u(:,i-1);
    err(:,i-1) = k*R{i-1}'*cross(y_est,y(:,i-1));
    R{i} = R{i-1}*expm(dt*skew(err(:,i-1)));
    
    
end
