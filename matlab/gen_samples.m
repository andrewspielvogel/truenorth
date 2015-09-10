function samp = gen_samples(hz,time,lat_deg)

lat    = lat_deg*pi/180;
t      = 0:1/hz:time;

num_samples = size(t,2);
g = 9.81;
r_e  = 6371*1000*[cos(lat);0;sin(lat)];
g_e  = g*r_e/norm(r_e);
w_se = [0;0;15*pi/(3600*180)];


w_ni = 0*[zeros(1,num_samples);zeros(1,num_samples);ones(1,num_samples)*(5*pi/180)];
a_n  = 0*[zeros(1,num_samples);zeros(1,num_samples);-sin(t)];
v_n  = 0*[zeros(1,num_samples);zeros(1,num_samples);cos(t)];
x_n  = 0*[zeros(1,num_samples);zeros(1,num_samples);sin(t)];



R_se = cellfun(@(A) expm(skew(w_se)*A),num2cell(t),'UniformOutput',false);
R_en = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];
R_ni = cellfun(@(A,B) expm(skew(A)*B),num2cell(w_ni,1),num2cell(t),'UniformOutput',false);
R_sn = cellfun(@(A) A*R_en,R_se,'UniformOutput',false);
R_si = cellfun(@(A,B) A*B,R_sn,R_ni,'UniformOutput',false);

w_i_pure = cellfun(@(A,B) A'*B,R_ni,num2cell(w_ni,1),'UniformOutput',false);
w_pure   = cellfun(@(A,B) A+B'*w_se,w_i_pure,R_si,'UniformOutput',false);

% noise sigs
w_sig = (2/(10000*sqrt(1/hz)))*pi/180;
a_sig = .12*sqrt(3)/(1000*sqrt(1/hz));

% bias sigs
w_bias_sig = (.1/3600)*pi/180;
a_bias_sig = .05/1000;

a_veh  = cellfun(@(A,B) A*B,R_sn,num2cell(a_n,1),'UniformOutput',false);
a_cent = cellfun(@(A,B) 2*skew(w_se)*A*B,R_sn,num2cell(v_n,1),'UniformOutput',false);
a_cor1 = cellfun(@(A,B) skew(w_se)*skew(w_se)*(A*B),R_sn,num2cell(x_n,1),'UniformOutput',false);
a_cor  = cellfun(@(A,B) skew(w_se)*skew(w_se)*A*r_e+B,R_se,a_cor1,'UniformOutput',false);

a_tot1 = cellfun(@(A,B) A+B,a_veh,a_cent,'UniformOutput',false);
a_tot2 = cellfun(@(A,B) A+B,a_tot1,a_cor,'UniformOutput',false);
a_tot  = cellfun(@(A,B) A+B*g_e,a_tot2,R_se,'UniformOutput',false);
a_pure = cellfun(@(A,B) A'*B,R_si,a_tot,'UniformOutput',false);

w_noise = w_bias_sig*randn(num_samples,3) + w_sig*randn(num_samples,3);
a_noise = a_bias_sig*randn(num_samples,3) + a_sig*randn(num_samples,3);

%samp.w_noise  = w_noise;
%samp.a_noise  = a_noise;
% samp.w_i_pure = w_i_pure;
% samp.w_pure   = w_pure;
% samp.g_s      = g_s;
% samp.a_veh    = a_veh;
% samp.a_cor    = a_cor;
% samp.a_cent   = a_cent;
% samp.a_pure   = a_pure;
% samp.a_tot    = a_tot;
% samp.a_tot2   = a_tot2;
% samp.R_se     = R_se;
% samp.R_en     = R_en;
% samp.R_ni     = R_ni;
samp.R_si     = R_si;
samp.t        = t;

% generate samples
samp.ang = cell2mat(w_pure)'   + w_noise;
samp.acc = cell2mat(a_pure)'/g + a_noise/3;
