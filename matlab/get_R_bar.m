function [g_s,R_sn,R_delta,g_delta,R_bar] = get_R_bar(w_m,a_m,t,lat)

lat = lat*pi/180;

% define earth rotation
w_se = [0;0;15*pi/(3600*180)];

Rd = get_R_delta(w_m,t);

g = r_lat_long(lat);

north = [-sin(lat);0;cos(lat)];
east  = [0;1;0];
down  = [-cos(lat);0;-sin(lat)];

R_en  = [north,east,down];


whats = cellfun(@(A) skew(w_se)*A,num2cell(t),'UniformOutput',false);
R_se = cellfun(@(A) expm(A),whats,'UniformOutput',false);
g_s = cellfun(@(A) A*g,R_se,'UniformOutput',false);

R_sn = R_se{end}*R_en;

R_delta = reshape(Rd',3,3,size(Rd,1));
g_d_mat = bsxfun(@times,R_delta,reshape(a_m',1,3,numel(a_m)/3));

g_d = sum(g_d_mat,2);

g_delta = reshape(g_d,3,numel(g_d)/3);
gs_mat = cell2mat(g_s);
g_s = reshape(gs_mat,3,numel(gs_mat)/3);

R_bar = g_s*g_delta'*inv(g_delta*g_delta');

end