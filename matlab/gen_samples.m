function samp = gen_samples(hz,time,lat_deg)

lat    = lat_deg*pi/180;
t      = 0:1/hz:time;

% init variables
num_samples = size(t,2);
g = 9.81;
r_e  = 6371*1000*[cos(lat);0;sin(lat)];
g_e  = g*r_e/norm(r_e);
w_se = [0;0;15*pi/(3600*180)];

% define instrument angular vel in ned frame and the x,v,a of the
% instrument in ned frame

w_ni = 0*[zeros(1,num_samples);zeros(1,num_samples);ones(1,num_samples)*(5*pi/180)];
a_n  = 0*[zeros(1,num_samples);zeros(1,num_samples);-sin(t)];
v_n  = 0*[zeros(1,num_samples);zeros(1,num_samples);cos(t)];
x_n  = 0*[zeros(1,num_samples);zeros(1,num_samples);sin(t)];


% generate R_se (space to Earth frame) matrices for ts
R_se = cellfun(@(A) expm(skew(w_se)*A),num2cell(t),'UniformOutput',false);

% generate R_en (earth to NED frame) based on lat
R_en = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];

% generate R_ni (NED to instrument) matrices based on w_ni (right now w_ni is zero---NEED to update code
% for non-constant w_ni
R_ni = cellfun(@(A,B) expm(skew(A)*B),num2cell(w_ni,1),num2cell(t),'UniformOutput',false);

% generate R_sn (Space to NED frame) matrices
R_sn = cellfun(@(A) A*R_en,R_se,'UniformOutput',false);

% generate R_si (Space to Instrument frame) matrices
R_si = cellfun(@(A,B) A*B,R_sn,R_ni,'UniformOutput',false);

% generate w pure signal in instrument frame --- w_pure = R_in*w_ni+R_is*w_se
w_i_pure = cellfun(@(A,B) A'*B,R_ni,num2cell(w_ni,1),'UniformOutput',false);
w_pure   = cellfun(@(A,B) A+B'*w_se,w_i_pure,R_si,'UniformOutput',false);

% noise sigmas CHECK THIS
w_sig_spec = (2/(10000*sqrt(1/hz)))*pi/180;  % units are rad/sec
a_sig_spec = 0.12*sqrt(3)/(1000*sqrt(1/hz)); % units are g, not m/s^2

w_sig = 8.4178 * 10^(-5);  % measured 1775, units are rad/sec
a_sig = 0.0023;            % measured 1775, units are g, not m/s^2

% bias sigmas 
% w_bias_sig = (.1/3600)*pi/180;
% a_bias_sig = .05/1000;
w_bias_sig = 0;
a_bias_sig = 0;

% generate acc component in star frame due to instrument a in ned frame --- R_sn*a_n
a_veh  = cellfun(@(A,B) A*B,R_sn,num2cell(a_n,1),'UniformOutput',false);

% generate acc component in star frame due to cent acc --- 2*skew(w_se)*R_sn*v_n
a_cent = cellfun(@(A,B) 2*skew(w_se)*A*B,R_sn,num2cell(v_n,1),'UniformOutput',false);

% generate acc component in star frame due to cor ---
% skew(w_se)^2*(R_se*r_e+R_sn*x_n
a_cor1 = cellfun(@(A,B) skew(w_se)*skew(w_se)*(A*B),R_sn,num2cell(x_n,1),'UniformOutput',false);
a_cor  = cellfun(@(A,B) skew(w_se)*skew(w_se)*A*r_e+B,R_se,a_cor1,'UniformOutput',false);

% generate pure acc signal in instrument frame --- R_is*(a_veh+a_cent+a_cor+R_se*g_e)
a_tot1 = cellfun(@(A,B) A+B,a_veh,a_cent,'UniformOutput',false);
a_tot2 = cellfun(@(A,B) A+B,a_tot1,a_cor,'UniformOutput',false);
a_tot  = cellfun(@(A,B) A+B*g_e,a_tot2,R_se,'UniformOutput',false);
a_pure = cellfun(@(A,B) A'*B,R_si,a_tot,'UniformOutput',false);

% generate noise
w_noise = w_bias_sig*randn(num_samples,3) + w_sig*randn(num_samples,3);
a_noise = a_bias_sig*randn(num_samples,3) + a_sig*randn(num_samples,3);

% store working variables in output structure
samp.num_samples = num_samples;
samp.g = g;
samp.r_e = r_e;
samp.g_e = g_e;
samp.w_se = w_se;

samp.w_sig = w_sig;
samp.a_sig = a_sig;

samp.w_noise  = w_noise;
samp.a_noise  = a_noise;
samp.w_i_pure = w_i_pure;
samp.w_pure   = w_pure;
% samp.g_s    = g_s;
samp.a_veh    = a_veh;
samp.a_cor1   = a_cor1;
samp.a_cor    = a_cor;
samp.a_cent   = a_cent;
samp.a_pure   = a_pure;
samp.a_tot    = a_tot;
samp.a_tot2   = a_tot2;
samp.R_se     = R_se;
samp.R_sn     = R_sn;
samp.R_en     = R_en;
samp.R_ni     = R_ni;
samp.R_si     = R_si;
samp.t        = t;

% generate samples
samp.ang = cell2mat(w_pure)'   + w_noise;
samp.acc = cell2mat(a_pure)' *(1.0/g) + a_noise;
