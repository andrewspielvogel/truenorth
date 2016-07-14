function out = adap_so3(lat,samp,R0,Rd)

num_samp = size(samp.acc,2);
R{num_samp} = eye(3);
R{1} = R0;

a_true  = zeros(3,num_samp);
a_est   = zeros(3,num_samp);
a_error = zeros(3,num_samp);

e_true  = zeros(3,num_samp);
e_est   = zeros(3,num_samp);
e_est_n   = zeros(3,num_samp);
e_est_z   = zeros(3,num_samp);
e_est_i   = zeros(3,num_samp);
e_error = zeros(3,num_samp);
e_m = zeros(3,num_samp);

dacc = zeros(3,num_samp);
acc_est = zeros(3,num_samp);

acc_est(:,1) = samp.acc(:,1);

e_est_z_avg = zeros(3,num_samp);


k_g = 1;
k_a = 1;
k_w = .0001;
k_e = .1;

for i=2:num_samp
    
    %dt = samp.t(i) - samp.t(i-1);
    dt = 1/5000;
    Rsn = get_Rsn(lat,samp.t(i-1));
    
    % 1st order filter acc signal
    da = samp.acc(:,i-1) - acc_est(:,i-1);
    
    acc_est(:,i) = acc_est(:,i-1) + k_a*da*dt;
    
    % gravity vector estimation
    a_true(:,i-1) = Rsn*[0;0;-1];
    a_est(:,i-1)  = R{i-1}*Rd{i-1}*acc_est(:,i-1);
    
    a_error(:,i-1) = k_g*R{i-1}'*cross(a_est(:,i-1),a_true(:,i-1));
    
    
    % east vector estimation in t_zero frame
    e_true(:,i-1) = Rsn*[0;1;0];    
    
    dacc(:,i) = (acc_est(:,i)-acc_est(:,i-1))/dt;
    e_m(:,i-1) = skew(samp.ang(:,i-1))*acc_est(:,i-1);
    e_est_i(:,i-1) = e_m(:,i-1)  + dacc(:,i-1);
    e_est_z(:,i-1) = Rd{i-1}*e_est_i(:,i-1);
    
    
    % 1st order filter east estimation signal
    de = e_est_z(:,i-1) - e_est_z_avg(:,i-1);
    e_est_z_avg(:,i) = e_est_z_avg(:,i-1) + k_e*dt*de;
            
    
    % heading updat law
    e_est(:,i-1) = R{i-1}*e_est_z_avg(:,i);
    
    e_est_n(:,i-1) = e_est(:,i-1)/norm(e_est(:,i-1));
    
    e_err = cross(e_est_n(:,i-1),e_true(:,i-1));
    
    err = dot(e_err,a_true(:,i-1))*a_true(:,i-1);
    
    e_error(:,i-1) = k_w*R{i-1}'*err;
    
    
    dR_a = expm(skew(a_error(:,i-1))*dt);
    dR_w = expm(skew(e_error(:,i-1))*dt);
    
    
    R{i} = R{i-1}*dR_a*dR_w;
    
    
end

out.R = R;
out.a_true  = a_true;
out.a_est   = a_est;
out.a_error = a_error;
out.e_true  = e_true;
out.e_est   = e_est;
out.e_est_n   = e_est_n;
out.e_est_z   = e_est_z;
out.e_est_i   = e_est_i;
out.e_error = e_error;
out.dacc = dacc;
out.acc_est = acc_est;
out.e_m = e_m;
out.e_est_z_avg = e_est_z_avg;



function R = get_Rse(t)

rate = 15*pi/180/3600;

Rse = expm(skew([0,0,1])*rate*t);

R = Rse;

function R = get_Rsn(lat,t)

Ren = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];
Rse = get_Rse(t);

R = Rse*Ren;
