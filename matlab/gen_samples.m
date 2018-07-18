function samp = gen_samples(lat,hz,t_end,bias)

dt = 1/hz;
t= 0:dt:t_end;
num = size(t,2);

lat = lat*pi/180;

Rni{num} = eye(3);
Rni{1} = eye(3);

ang = zeros(3,num);
acc = zeros(3,num);
samp.E = zeros(3,num);


T = [0.991,-0.002,-0.005;0,0.996,0.018;0,0,1.014];

% noise
% w_sig = 0.005; %6.32 * 10^(-3)*pi/180;  % measured 1775, units are rad/sec
% a_sig = 0.00065; %0.0037;            % measured 1775, units are g, not m/s^2
% m_sig = 0.001; %0.002; % units are gauss


w_sig = 6.32 * 10^(-3)*pi/180;  % measured 1775, units are rad/sec
a_sig = 0.0037;            % measured 1775, units are g, not m/s^2
m_sig = 0.002; % units are gauss

% generate a_n
r = 6371*1000;
Ren = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];
a_e = [cos(lat);0;sin(lat)] - (15.04*pi/180/3600)^2*cos(lat)*[r;0;0]/9.81;
a_n = Ren'*a_e;

m_n = [0.205796;-0.040654;0.468785];

w_E_e = [0;0;1]*15.04*pi/180/3600;
w_E_n = Ren'*w_E_e;
w_E_n
fileID = fopen('/home/spiels/log/tn/noise/exp2.KVH','w');

for i=1:num


    % get w_v, Rsn
    w_veh = get_w(t(i));

    if i~=num
        
        Rni{i + 1} = Rni{i}*expm(skew(w_veh)*dt);
    end

    ang(:,i) = w_veh + Rni{i}'*w_E_n + bias.ang + w_sig*randn(3,1);
    acc(:,i) = Rni{i}'*a_n + bias.acc + a_sig*randn(3,1);%+skew(w_veh)*[0.1;0;0];
    samp.acc_nv(:,i) = acc(:,i);
    samp.E(:,i) = Rni{i}'*[0;1;0];
    samp.D(:,i) = Rni{i}'*a_n;
    samp.att(:,i) = rot2rph(Rni{i});
    samp.w_E(:,i) = Rni{i}'*w_E_n;
    samp.w_E_n(:,i) = Rni{i}'*[w_E_n(1);0;0];
    %T=eye(3);
    %samp.mag(:,i) = T*Rni{i}'*m_n + m_sig*randn(3,1) + bias.mag;
    samp.mag(:,i) = Rni{i}'*m_n + m_sig*randn(3,1) + bias.mag;
    
    % print progress
    if ~mod(t(i),30)
        str = sprintf('Made %i:%i0 of data at %i hz',floor(t(i)/60),mod(t(i),60)/10,hz);
        disp(str);
    end
    R = Rni{i};
    rpy_phins = samp.att(:,i);%rot2rph(R);
    %fprintf(fileID,'IMU_RAW, %.40f,%.40f,%.40f, %.35f,%.35f,%.35f,%f,%f,%f, 0, 0, %.30f,0,1,1,1,1,1,1, %f,%f,%f,%f,%f,%f,%f,%f,%f,%.40f,%.40f,%.40f \n',ang(1,i),ang(2,i),ang(3,i),acc(1,i),acc(2,i),acc(3,i),samp.mag(1,i),samp.mag(2,i),samp.mag(3,i),t(i),R(1,1),R(1,2),R(1,3),R(2,1),R(2,2),R(2,3),R(3,1),R(3,2),R(3,3),rpy_phins(1),rpy_phins(2),rpy_phins(3));
    fprintf(fileID,'IMU_RAW, %.40f,%.40f,%.40f, %.35f,%.35f,%.35f,%f,%f,%f, 0,%d,%.5f,0,1,1,1,1,1,1,%f,%f,%f \n',ang(1,i),ang(2,i),ang(3,i),acc(1,i),acc(2,i),acc(3,i),samp.mag(1,i),samp.mag(2,i),samp.mag(3,i),mod(i,128),t(i),rpy_phins(1),rpy_phins(2),rpy_phins(3));

end

samp.t = t;
samp.acc = acc;
samp.ang = ang;
samp.Rni = Rni;
samp.bias = bias;


function w = get_w(t)

%%%% IROS2018

% w = [cos(t/2)/20;sin(t/5)/15;-cos(t/30)/10]*0;
% 
% w = [cos(t/50)/25;-sin(t/9)/7*0;cos(t/10)/6]; %exp1
% w = [cos(t/50)/25*0;-sin(t/9)/7*0;cos(t/10)/6]; %exp2
% w = [cos(t/50)/100;-sin(t/9)/7*0;cos(t/10)/6]; %exp3
% 
% w = [cos(t/50)/25;-sin(t/25)/10;cos(t/10)/5]; %exp4 sim2
% 
% %w = [cos(t/20)/55;-sin(t/5)/30;cos(t/6)/2]; %exp5 sim1

w = [cos(t/20)/55;-sin(t/5)/30;cos(t/6)/2];
w = [cos(t/50)/25;-sin(t/25)/10;cos(t/10)/5];
w = [0;0;cos(t/120)/30];


if t>1500
    %w=[0;0;sin(t/20)/10];
end