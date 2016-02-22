function R_in = getstartR(oct,samp,recon,bias)
lat = 39.299*pi/180;
% generate R_en (earth to NED frame) based on lat
R_en = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];
diff = abs(oct.t-samp.stamp(1));
[idx idx] = min(diff);
R_ni = rph2R(oct.gyro_attitude(idx,:)*pi/180)';


w_se = [0;0;15*pi/(3600*180)];


w = mean(samp.ang,1)-bias.ang;
a = mean(samp.acc,1)-bias.acc;

w = w';
a = a';

d = -a;
e = cross(d,w);
n = cross(e,d);

n = n/norm(n);
e = e/norm(e);
d = d/norm(d);

R_ni = [n, e, d]';
Rb = R_en*R_ni;

R_se = cellfun(@(A) expm(skew(w_se)*(A-samp.stamp(1))),num2cell(samp.stamp),'UniformOutput',false);



% generate R_sn (Space to NED frame) matrices
R_sn = cellfun(@(A) A*R_en,R_se,'UniformOutput',false);

% R_si (space to instrument frame) matrices recovered from data

R_si = cellfun(@(A) Rb*A,recon.Rdelta,'UniformOutput',false);

% recovered (instrument to ned frame) matrices from data
R_in = cellfun(@(A,B) A'*B,R_si,R_sn','UniformOutput',false);
