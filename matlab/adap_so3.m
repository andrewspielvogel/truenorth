function out = adap_so3(lat,samp,R0,Rd)

num_samp = size(samp.acc,2);
R{num_samp} = eye(3);
R{1} = R0;

a_true  = zeros(3,num_samp);
a_est   = zeros(3,num_samp);
a_error = zeros(3,num_samp);

e_true  = zeros(3,num_samp);
e_est   = zeros(3,num_samp);
e_error = zeros(3,num_samp);


k_a = 1;
k_w = 500;
k_w2 = 30;

for i=2:num_samp
    
    if samp.stamp(i)> 60*3
        k_w = k_w2;
    end
    
    dt = samp.stamp(i) - samp.stamp(i-1);
    
    a_true(:,i-1) = get_Rsn(lat,samp.t(i-1))*[0;0;-1];
    a_est(:,i-1)  = R{i-1}*Rd{i-1}*samp.acc(:,i-1);
    
    a_error(:,i-1) = k_a*R{i-1}'*cross(a_est(:,i-1),a_true(:,i-1));
    
    
    e_true(:,i-1) = get_Rsn(lat,samp.t(i-1))*[0;1;0];
    e_est(:,i-1)  = R{i-1}*Rd{i-1}*skew(samp.ang(:,i-1))*samp.acc(:,i-1);
    
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
out.e_error = e_error;


function R = get_Rse(t)

rate = 15*pi/180/3600;

Rse = expm(skew([0,0,1])*rate*t);

R = Rse;

function R = get_Rsn(lat,t)

Ren = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];
Rse = get_Rse(t);

R = Rse*Ren;
