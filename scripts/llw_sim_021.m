% 2018-10-09 LLW sentry494 survey portion only, 18 params w/constant
% bias, param ICs from llw_sim_017, with RMS computed for entire
% interval and estimator initialized to Phins attitude.

options = optimset('PlotFcns',@optimplotfval);

% param IC from results of llw_sim_017
Parameter_IC = [ 1.797075942718001   0.476381971548217   0.115529314082092   0.966094683883134   0.002966929623844   0.035671914097419   4.620159576563259   6.564472085970264   5.909908052081234   0.000078100412606   0.001045090127745  0.000073934833181   0.000044680489012   0.000427157486436   0.000014766234763   0.011561540910567   0.786668932474017  -0.011925422840177];
							   
Parameter_IC'

% keep all gains nonnegative, but no constraint on boas terms
Lower_Bound = [-1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1 -1 -1 -1 -1 -1];

Lower_Bound'

result = fmincon('tn_opt_llw_494_survey', Parameter_IC, [], [], [], [], Lower_Bound, [], [], options)

% name for figures
figsname = "llw_sim_021_sentry494_survey_only_18_param_const_bias"

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

% save the workspace
save([figsname '.mat'])
