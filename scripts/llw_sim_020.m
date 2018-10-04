% 2018-10-03 LLW sentry492 first 30 min of survey portion only, params from llw_sim_014

options = optimset('PlotFcns',@optimplotfval);

Parameter_IC = [    1.380652837746997   0.347963733907383   0.003919055966846   0.890206902296098   0.000026202845260   0.000170907157384   4.881989142851892  12.081061391754407   5.511298694389958   0.000060376732873   0.000091606987264   0.000039745794239   0.000009228707961   0.000033856518434   0.000010994226174   0.024896646677126  -0.135650547605661  -0.008397191025680];

Parameter_IC'

% keep all gains nonnegative, but no constraint on boas terms
Lower_Bound = [-1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1 -1 -1 -1 -1 -1];

Lower_Bound'

result = fmincon('tn_opt_llw_494_survey_first_30', Parameter_IC, [], [], [], [], Lower_Bound, [], [], options)

% name for figures
figsname = "llw_sim_019_sentry492_survey_only_first_30_min_18_param_const_bias"

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
save(sprintf('%s.mat',figsname))
