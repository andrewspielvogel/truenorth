function out = adap_so3(y_a,u_a,y_w,u_w,t,R0,Rd)

num_samp = size(u_a,2);
R{num_samp} = eye(3);
R{1} = R0;
err_a = zeros(3,num_samp);
err_w = zeros(3,num_samp);
y_est_a = zeros(3,num_samp);
cor_w = zeros(3,num_samp);
w_s = zeros(3,num_samp);
w_a = zeros(3,num_samp);
a_est = zeros(3,num_samp);

k_a = 1;
k_w1 = 500;
k_w2 = 50;

for i=2:num_samp
    
    dt = t(i)-t(i-1);
    y_est_a(:,i-1) = R{i-1}*u_a(:,i-1);
    err_a(:,i-1) = k_a*R{i-1}'*cross(y_est_a(:,i-1),y_a(:,i-1));
    
    if num_samp > 60*5000*5
        k_w = k_w2;
    else
        k_w = k_w1;
    end
    
    
    a_est(:,i-1) = Rd{i-1}'*R{i-1}'*y_a(:,i-1);
    
    
    w_a(:,i-1) = -skew(a_est(:,i-1))*skew(a_est(:,i-1))*u_w(:,i-1);
    
    w_s(:,i-1) = R{i-1}*Rd{i-1}*w_a(:,i-1);
    
    err_w(:,i-1) = cross(w_s(:,i-1),y_w(:,i-1));
    
    cor_w(:,i-1) = k_w*R{i-1}'*err_w(:,i-1);
    
    R{i} = R{i-1}*expm(dt*skew(err_a(:,i-1)+cor_w(:,i-1)));
    
    
end
out.R = R;
out.y_est_a = y_est_a;
out.err_a = err_a;
out.cor_w = cor_w;
out.err_w = err_w;
out.w_s = w_s;
out.w_a = w_a;
out.a_est = a_est;