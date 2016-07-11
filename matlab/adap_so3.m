function out = adap_so3(lat,samp,R0,Rd)

num_samp = size(samp.acc,2);
R{num_samp} = eye(3);
R{1} = R0;

a_true  = zeros(3,num_samp);
a_est   = zeros(3,num_samp);
a_error = zeros(3,num_samp);

e_true  = zeros(3,num_samp);
e_est   = zeros(3,num_samp);
e_est_z   = zeros(3,num_samp);
e_est_i   = zeros(3,num_samp);
e_error = zeros(3,num_samp);
e_m = zeros(3,num_samp);

dacc = zeros(3,num_samp);
acc_est = zeros(3,num_samp);

acc_est(:,1) = samp.acc(:,1);

e_est_z_avg = zeros(3,num_samp);


k_a = 1;
k_w = 1000;%.005
k_w2 = 100;
k_e = .01;

for i=2:num_samp
    
    if samp.t(i) > 2*60
        k_w = k_w2;
    end
    
    dt = samp.t(i) - samp.t(i-1);
    
    Rsn = get_Rsn(lat,samp.t(i-1));
    
    
    % gravity vector estimation
    a_true(:,i-1) = Rsn*[0;0;-1];
    a_est(:,i-1)  = R{i-1}*Rd{i-1}*samp.acc(:,i-1);
    
    a_error(:,i-1) = k_a*R{i-1}'*cross(a_est(:,i-1),a_true(:,i-1));
    
    
    % east vector estimation
    e_true(:,i-1) = Rsn*[0;1;0];
    
    da = samp.acc(:,i-1) - acc_est(:,i-1);
    
    acc_est(:,i) = acc_est(:,i-1) + da*dt;
    
    dacc(:,i) = (acc_est(:,i)-acc_est(:,i-1))/dt;
    e_m(:,i-1) = skew(samp.ang(:,i-1))*acc_est(:,i-1);
    e_est_i(:,i-1) = e_m(:,i-1)  + dacc(:,i-1);
    e_est_z(:,i-1) = Rd{i-1}*e_est_i(:,i-1);
    
    de = e_est_z(:,i-1) - e_est_z_avg(:,i-1);
    e_est_z_avg(:,i) = e_est_z_avg(:,i-1) + k_e*dt*de;
        
    e_est(:,i-1) = R{i-1}*e_est_z_avg(:,i-1);
    
    e_error(:,i-1) = k_w*R{i-1}'*cross(e_est(:,i-1),e_true(:,i-1));
    
    
    dR = expm(skew(a_error(:,i-1)+e_error(:,i-1))*dt);
    
    R{i} = R{i-1}*dR;
    
    
end

out.R = R;
out.a_true  = a_true;
out.a_est   = a_est;
out.a_error = a_error;
out.e_true  = e_true;
out.e_est   = e_est;
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
