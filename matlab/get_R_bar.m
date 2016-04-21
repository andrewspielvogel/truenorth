function out = get_R_bar(samp,bias,lat,cut_freq_hz_a,cut_freq_hz_w,samp2avg)

samp.ang = samp.ang';
samp.acc = samp.acc';

num_samples = size(samp.ang,1);
 if ~exist('samp2avg','var')
     % third parameter does not exist, so default it to something
      samp2avg = num_samples;
 end
samp.ang = samp.ang - repmat(bias.ang',num_samples,1);
samp.acc = samp.acc - repmat(bias.acc',num_samples,1);

samp.stamp = samp.stamp';
reconstruct.t = samp.stamp;
samp.acc = my_lowpass(samp.acc,samp.hz,1,cut_freq_hz_a);
samp.ang = my_lowpass(samp.ang,samp.hz,1,cut_freq_hz_w);

earth_rad   = 6371*1000; % radius in meters
lat  = lat*pi/180; % convert to radians
w_se = [0;0;15*pi/(3600*180)]; % earth rate in rad/sec in space frame


% get R_delta (pass in samples and the initial condition)
Rd   = get_R_d(samp,eye(3));

% define g and earth radius in earth frame
g = 9.81;
g_e    = g*r_lat_long(lat); %  gravity vector in earth frame m/s
r_e  = earth_rad*r_lat_long(lat); % meters

% g_s = Rbar*g_d
% generate analytical g in star frame using lat
% NEED TO UPDATE FOR GENERAL CASE -- ASSUMING pos(x),vel(v),acc(a) of
% instrument in NED frame are zero
%g_s = cellfun(@(A) expm(skew(w_se)*A)*g,num2cell(samp.t),'UniformOutput',false);
g_s  = cellfun(@(A) skew(w_se)*skew(w_se)*expm(skew(w_se)*(A-samp.stamp(1)))*r_e+expm(skew(w_se)*A)*g_e,num2cell(samp.stamp),'UniformOutput',false);

% transform measured acc into g vectors in zero frame (gd = Rdelta*a_m)
g_d = cellfun(@(A,B) A*B',Rd,num2cell(samp.acc,2)','UniformOutput',false);


reconstruct.gs = cell2mat(g_s)/g; %normalize to be in units of g
reconstruct.gd = cell2mat(g_d);
reconstruct.Rdelta = Rd;

% find Rbar using svd method
%reconstruct.Rbar = fit_2_sets(reconstruct.gs,reconstruct.gd);

%%%%%

% generate east approx from samples e(i) = Rdelta(i)*(-skew(a_m(i))*w_m(i)
e1 = cellfun(@(A,B) A*skew(B),reconstruct.Rdelta(1:samp2avg),num2cell(samp.ang(1:samp2avg,:),2)','UniformOutput',false);
e_raw = cellfun(@(A,B) A*B',e1,num2cell(samp.acc(1:samp2avg,:),2)','UniformOutput',false);
e = cellfun(@(A) A/norm(A),e_raw,'UniformOutput',false);

% generate north approx from samples n(i) =
% Rdelta(i)*(-skew(a_m(i))*skew(a_m(i)))*w_m(i)
n1 = cellfun(@(A,B) (-1)*A*skew(B)*skew(B),reconstruct.Rdelta(1:samp2avg),num2cell(samp.acc(1:samp2avg,:),2)','UniformOutput',false);
n_raw = cellfun(@(A,B) A*B', n1,num2cell(samp.ang(1:samp2avg,:),2)','UniformOut',false);
n = cellfun(@(A) A/norm(A), n_raw,'UniformOut',false);

% generate R_se (space to Earth frame) matrices for ts
R_se = cellfun(@(A) expm(skew(w_se)*(A-samp.stamp(1))),num2cell(samp.stamp),'UniformOutput',false);

% generate analytical east and north vectors for case at equator (still need to generalize to all lat) 
d_a = cellfun(@(A) A*(-1)*r_lat_long(lat),R_se(1:samp2avg),'UniformOutput',false);
e_a = cellfun(@(A) A*[0;1;0],R_se(1:samp2avg),'UniformOutput',false);
n_a = cellfun(@(A,B) cross(A,B),e_a,d_a,'UniformOutput',false);


n_a_mat = cell2mat(n_a);
e_a_mat = cell2mat(e_a);
d_a_mat = cell2mat(d_a);

% normalize and create nde matrices for measured and analytical
reconstruct.a_s = [reshape(n_a_mat,3,size(n_a_mat,1)/3),reshape(e_a_mat,3,size(e_a_mat,1)/3),reshape(d_a_mat,3,size(d_a_mat,1)/3)];

reconstruct.m_s = [cell2mat(n),cell2mat(e),(-1)*reconstruct.gd(:,1:samp2avg)./repmat(sqrt(sum(reconstruct.gd(:,1:samp2avg).^2,1)),3,1)];


% find Rbar with svd method
reconstruct.rb = fit_2_sets(reconstruct.a_s,reconstruct.m_s);
reconstruct.acc = samp.acc;
reconstruct.ang = samp.ang;

% generate R_en (earth to NED frame) based on lat
R_en = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];

% generate R_sn (Space to NED frame) matrices
R_sn = cellfun(@(A) A*R_en,R_se,'UniformOutput',false);

% R_si (space to instrument frame) matrices recovered from data

reconstruct.R_si = cellfun(@(A) reconstruct.rb*A,Rd,'UniformOutput',false);

% recovered (instrument to ned frame) matrices from data
reconstruct.R_in = cellfun(@(A,B) A'*B,reconstruct.R_si,R_sn','UniformOutput',false);

out.Rin = reconstruct.R_in;
out.att = rph(reconstruct.R_in);
out.t = reconstruct.t;
out.rb = reconstruct.rb;
out.Rdelta = reconstruct.Rdelta;
out.Rsn = R_sn;

end