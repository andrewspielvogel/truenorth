function out = att_est( samp )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

kg = 1;
kw = .01;

hz = 5000;
lat = 39.32*pi/180;

R_align = [1,0,0;0,-1,0;0,0,-1];

num = size(samp.t,2);
num = hz*60*8.5;
out.t(1) = samp.t(1);

for i=2:num
    
    diff = samp.seq_num(i)-samp.seq_num(i-1);
    if diff<0
       
       diff = diff + 127;
        
    end
    out.t(i) = samp.t(i-1) + (1/hz)*diff;
    
end

acc = my_lowpass(samp.acc',hz,1,1)';
ang = my_lowpass(samp.ang',hz,1,1)';


up_n = [0;0;-1];
east_n = [0;1;0];

acc_true_s = zeros(3,num);
acc_est_s = zeros(3,num);
g_error = zeros(3,num);
east_est = zeros(3,num);
east_est_z = zeros(3,num);
east_error_s = zeros(3,num);
east_error = zeros(3,num);
dacc = zeros(3,num);
att = zeros(3,num);

Rb{1} = get_Rsn(lat,0)*R_align;
Rb{num} = eye(3);
Rd{1} = eye(3);
Rd{num} = eye(3);
Rni{1} = eye(3);
Rni{num} = eye(3);

for i=2:num
    
    dt = out.t(i) - out.t(i-1);
   
    R_sn = get_Rsn(lat,out.t(i)-out.t(1));
    
    acc_true_s(:,i) = R_sn*up_n;
    acc_est_s(:,i) = Rb{i-1}*Rd{i-1}*acc(:,i);
    
    g_error(:,i) = kg*Rb{i-1}'*skew(acc_est_s(:,i))*acc_true_s(:,i);
    
    
    
    dacc(:,i) = (acc(:,i)-acc(:,i-1))/dt;
       
    east_est(:,i) = skew(ang(:,i))*acc(:,i);
    east_est_z(:,i) = Rd{i-1}*(east_est(:,i-1) + dacc(:,i));
    
    east_est_z(:,i) = east_est_z(:,i)/norm(east_est_z(:,i));
    
    east_error_s(:,i) = skew(Rb{i-1}*east_est_z(:,i))*R_sn*east_n;
    east_error_s(:,i) = dot(east_error_s(:,i),acc_true_s(:,i))*acc_true_s(:,i);
    east_error(:,i) = kw*Rb{i-1}'*east_error_s(:,i);

    Rni{i-1} = R_sn'*Rb{i-1}*Rd{i-1};
    att(:,i) = rot2rph(Rni{i-1}*R_align);
    
    
    Rb{i} = Rb{i-1}*expm(skew(g_error(:,i)+east_error(:,i))*dt);
    Rd{i} = Rd{i-1}*expm(skew(ang(:,i)*dt));
    
    
end
out.acc = acc;
out.ang = ang;
out.acc_true_s = acc_true_s;
out.acc_est_s  = acc_est_s;
out.g_error    = g_error;
out.dacc       = dacc;
out.east_est   = east_est;
out.east_est_z = east_est_z;
out.east_error_s = east_error_s;
out.east_error = east_error;
out.Rb = Rb;
out.Rd = Rd;
out.att = att';
out.Rni = Rni;
out.stamp = out.t + samp.stamp(1);


function R = get_Rse(t)

rate = 15*pi/180/3600;

Rse = expm(skew([0,0,1])*rate*t);

R = Rse;

function R = get_Rsn(lat,t)

Ren = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];
Rse = get_Rse(t);

R = Rse*Ren;