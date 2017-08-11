function samp = gensamples(lat,hz,t_end,R_align,bias)
% gensamples(lat,hz,t_end,R_align,bias)

tic

lat = lat*pi/180;

if nargin<5
   
    bias.acc = zeros(3,1);
    bias.ang = zeros(3,1);
    bias.mag = zeros(3,1);
    
end


% Define t
dt = 1/hz;
t = 0:dt:t_end;

r = 6371*1000;

% noise
w_sig = 0*6.32 * 10^(-3)*pi/180;  % measured 1775, units are rad/sec
a_sig = 0*0.0037;            % measured 1775, units are g, not m/s^2
m_sig = 0.002;

num = size(t,2);


samp.ang = zeros(3,num);
samp.acc = zeros(3,num);
samp.att = zeros(3,num);
samp.att_v = zeros(3,num);
samp.mag = zeros(3,num);

samp.Rzi{1} = eye(3);

% generate a_n
Ren = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];
a_e = [cos(lat);0;sin(lat)] - (15.04*pi/180/3600)^2*cos(lat)*[r;0;0]/9.81;
a_n = Ren'*a_e;

m_n = [0.205;-0.041;0.470];

Rsz = get_Rsn(lat,0)*R_align;

samp.Rsz=Rsz;
    
fileID = fopen('/home/spiels/log/data2.KVH','w');

for i=1:num


    % get w_v, Rsn
    w_veh = get_w(t(i));
    Rsn = get_Rsn(lat,t(i));
    
    samp.Rsi{i} = Rsz*samp.Rzi{i};
    if i~=num
        
        w = samp.Rsi{i}'*[0;0;1]*15.04*pi/180/3600 + samp.Rsi{i}'*Rsn*w_veh;
        samp.Rzi{i + 1} = samp.Rzi{i}*expm(skew(w)*dt);
    end

    samp.ang_v(:,i) = samp.Rsi{i}'*Rsn*w_veh;
    samp.att(:,i) = rot2rph(Rsn'*samp.Rsi{i}*R_align');  
    samp.Rni{i} = Rsn'*samp.Rsi{i}*R_align';
    samp.mag(:,i) = samp.Rni{i}'*m_n + bias.mag + m_sig*randn(3,1);
    samp.ang(:,i) =  w + w_sig*randn(3,1) + bias.ang;
    samp.acc(:,i) =  samp.Rsi{i}'*Rsn*(a_n + get_a(t(i))) + a_sig*randn(3,1) + bias.acc;
    samp.acc_z(:,i) = samp.Rzi{i}*samp.acc(:,i);
    samp.acc_v(:,i) = samp.Rsi{i}'*Rsn*get_a(t(i));  
    samp.e_v(:,i) = skew(w_veh)*samp.acc(:,i);
    samp.E(:,i) =  samp.Rni{i}(:,2);
    


    % print progress
    if ~mod(t(i),30)
        str = sprintf('Made %i:%i0 of data at %i hz',floor(t(i)/60),mod(t(i),60)/10,hz);
        disp(str);
    end
    R = samp.Rni{i};
    fprintf(fileID,'IMU_RAW, %.40f,%.40f,%.40f, %.35f,%.35f,%.35f,%f,%f,%f, 0, 0, %.30f,0,1,1,1,1,1,1, %f,%f,%f,%f,%f,%f,%f,%f,%f \n',samp.ang(1,i),samp.ang(2,i),samp.ang(3,i),samp.acc(1,i),samp.acc(2,i),samp.acc(3,i),samp.mag(1,i),samp.mag(2,i),samp.mag(3,i),t(i),R(1,1),R(1,2),R(1,3),R(2,1),R(2,2),R(2,3),R(3,1),R(3,2),R(3,3));
    % fprintf(fileID,'IMU_RAW, %.40f,%.40f,%.40f, %.35f,%.35f,%.35f,%f,%f,%f, %d,%.5f,1,1,1,1,1,1 \n',samp.ang(1,i),samp.ang(2,i),samp.ang(3,i),samp.acc(1,i),samp.acc(2,i),samp.acc(3,i),samp.mag(1,i),samp.mag(2,i),samp.mag(3,i),mod(i,128),t(i));

end




% save
samp.t = t;
samp.stamp = t;
samp.hz = hz;
samp.bias = bias;
samp.noise.w_sig = w_sig;
samp.noise.a_sig = a_sig;
toc

function R = get_Rse(t)

rate = 15.04*pi/180/3600;

Rse = expm(skew([0,0,1])*rate*t);

R = Rse;

function R = get_Rsn(lat,t)

Ren = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];
Rse = get_Rse(t);

R = Rse*Ren;

function w = get_w(t)

if t>30*60
    w=[0;0;0];
else
    
%w = [0;1;0];
w = [cos(t/15)/100;sin(t/7)/60;-cos(t/20)/40];
%w = [sin(t/5)/70+cos(t/11)/40;cos(t/3)/50-sin(t/8)/20;sin(t/9)/30-cos(t/5)/50];

end

function a = get_a(t)

a = 0*[sin(t/5)/20;cos(t/5)/22;0];