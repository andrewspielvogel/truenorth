% 2018-09-10 LLW use fmincon to specify lower bound on all gains is zero
% LLW Sim #10 - fmincon start with 24 params where #8 concluded

options = optimset('PlotFcns',@optimplotfval);

% Parameter_IC = [  1.822104189331579   0.420549511242625   0.002328617829202  ...
%         	   0.788859606226511   0.000005892125440   0.000035254589027  ...
% 		   4.847289491570516  11.997485104915679   4.349747374315634  ...
% 		   0.000020797687328   0.000062954099449   0.000023368256065  ...
% 		   0.185405364353440   1.925763844110393   0.019586252609804  ...
% 		   0.000000122737953   0.000008372554092   0.000096566681871  ...
%		   0.000002704263508   0.000023786745511  -0.000085113438718  ...
%  	           0.021961154445260   0.016497893316138  -0.013948629059267]

% sim10 set IC for accel bias value to zero
Parameter_IC = [  1.822104189331579   0.420549511242625   0.002328617829202  ...
        	   0.788859606226511   0.000005892125440   0.000035254589027  ...
		   4.847289491570516  11.997485104915679   4.349747374315634  ...
		   0.000020797687328   0.000062954099449   0.000023368256065  ...
		   0 0 0 ...
		   0.000000122737953   0.000008372554092   0.000096566681871  ...
		   0.000002704263508   0.000023786745511  -0.000085113438718  ...
  	           0.021961154445260   0.016497893316138  -0.013948629059267]		

  
Parameter_IC'

% keep all gains nonnegative, but no constraint on boas terms
Lower_Bound = [-1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1 -1 -1 -1 -1 -1];

Lower_Bound'

result = fmincon('tn_opt_llw_sim010', Parameter_IC, [], [], [], [], Lower_Bound, [], [], options)

% name for figures
figsname = 'llw_sim_010_24_params_sentry492_redux'

% random string for figures
figs_random_string = random_string(10)

for fignum = 1:6
  % select figure
  figure(fignum);

  % add grid lines to optimplotfval
  if fignum == 6
    grid on;
  end 
  % construct filename
  filename = sprintf('%02d_%s_%s.pdf',fignum,figsname,figs_random_string);
  % create pdf
  orient landscape
  print('-dpdf', filename,'-fillpage');
end


% construct filename for combined pdfs
combined_pdf_filename = sprintf('%s_%s.pdf',figsname,datestr(now,'yyyy-mm-dd_HH-MM-SS')) 

% delete it if it exists already
if(exist(combined_pdf_filename))
  delete(combined_pdf_filename);
end

% combine individual pdfs
pdftk_command = ['pdftk *' figs_random_string '*pdf cat output ' combined_pdf_filename];
unix(pdftk_command);

% delete individual pdfs
delete(['*' figs_random_string '*pdf']); 

% open pdf viewer on combined file
unix(['evince '     combined_pdf_filename '&']);

