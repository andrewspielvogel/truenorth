function out = att_est( samp ,hz,  real)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

kg = 1;
kw = .05;
kerror = .01;

lat = 39.32*pi/180;

R_align = [1,0,0;0,-1,0;0,0,-1];
R_align = eye(3);

num = size(samp.t,2);
out.t(1) = samp.t(1);

if real
for i= 2:num
    
    diff = samp.seq_num(i)-samp.seq_num(i-1);
    if diff<0
       
       diff = diff + 128;
        
    end
    out.t(i) = out.t(i-1) + (1/hz)*diff;
    
end
else
    out.t = samp.t;
end


acc = samp.acc;
ang = samp.ang;

earth_radius = 6371*1000;
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


Rb{1} = get_Rsn(lat,0)*R_align*rph2R([pi/20;pi/20;pi/20]);
Rb{num} = eye(3);
Rd{1} = eye(3);
Rd{num} = eye(3);
Rni{1} = eye(3);
Rni{num} = eye(3);

out.east_est_s = zeros(3,num);
out.east_est_n = zeros(3,num);
out.east_est_n(:,1) = [0;1;0]*(15*pi/180/3600)*cos(lat);


out.east_error_n2 = zeros(3,num);

Ren = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];
a_e = ([cos(lat);0;sin(lat)] - (15*pi/180/3600)^2*cos(lat)*[earth_radius;0;0]/9.81);
a_e = a_e/norm(a_e);

a_n = Ren'*a_e;
out.Rsi{1} = Rb{1};
for i=2:num
    
    dt = out.t(i) - out.t(i-1);
   
    Rd{i} = Rd{i-1}*expm(skew(ang(:,i-1)*dt));
    
    R_sn = get_Rsn(lat,out.t(i)-out.t(1));
    

    acc_true_s(:,i) = get_Rse(out.t(i))*a_e;
    acc_est_s(:,i) = Rb{i-1}*Rd{i}*acc(:,i);
    out.acc_z(:,i) = Rd{i}*acc(:,i); 
    
    g_error(:,i) = kg*Rb{i-1}'*skew(acc_est_s(:,i))*acc_true_s(:,i);  
    
    dacc(:,i) =(acc(:,i)-acc(:,i-1))/dt;
    
    out.east(:,i) = skew(ang(:,i))*acc(:,i);
    east_est(:,i) = skew(ang(:,i))*acc(:,i) +dacc(:,i);
    east_est_z(:,i) = Rd{i-1}*(east_est(:,i));   

    
    out.east_est_s(:,i) = Rb{i-1}*east_est_z(:,i);
    east_est_z(:,i) = east_est_z(:,i)/norm(east_est_z(:,i));

    out.east_est_n(:,i) = R_sn'*out.east_est_s(:,i);
    out.east_est_n(:,i) = out.east_est_n(:,i-1)*exp(-dt*2*pi*kerror) + (1-exp(-dt*2*pi*kerror) )*out.east_est_n(:,i);
    out.east_est_n_norm(:,i) = out.east_est_n(:,i)/norm(out.east_est_n(:,i));

    out.east_error_n(:,i) = cross(out.east_est_n_norm(:,i),east_n);
    out.east_error_n(:,i) = dot(out.east_error_n(:,i),a_n)*a_n;
    
    east_error(:,i) = kw*Rb{i-1}'*east_error_s(:,i);
    east_error(:,i) = kw*Rb{i-1}'*R_sn*out.east_error_n(:,i);

    Rni{i-1} = R_sn'*Rb{i-1}*Rd{i};
    att(:,i) = rot2rph(Rni{i-1}*R_align);
    
    
    Rb{i} = Rb{i-1}*expm(skew(g_error(:,i)+east_error(:,i))*dt);
    
    out.Rsi{i} = Rb{i}*Rd{i};
    
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