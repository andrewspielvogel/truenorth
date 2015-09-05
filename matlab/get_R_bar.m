function reconstruct = get_R_bar(samp,lat)

earth_rad   = 6371*1000;
lat  = lat*pi/180;
w_se = [0;0;15*pi/(3600*180)];
Rd   = get_R_d(samp,eye(3));
g    = 9.81*r_lat_long(lat);
r_e  = earth_rad*[cos(lat);0;sin(lat)];

%g_s = cellfun(@(A) expm(skew(w_se)*A)*g,num2cell(samp.t),'UniformOutput',false);
g_s  = cellfun(@(A) skew(w_se)*skew(w_se)*expm(skew(w_se)*A)*r_e+expm(skew(w_se)*A)*g,num2cell(samp.t),'UniformOutput',false);
g_d = cellfun(@(A,B) A*B',Rd,num2cell(samp.acc,2)','UniformOutput',false);

reconstruct.Rdelta = Rd;
reconstruct.Rbar = fit_2_sets(cell2mat(g_d),cell2mat(g_s));

end