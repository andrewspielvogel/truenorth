% 2018-10-01 LLW sentry494 survey portion only, params from llw_sim_013

options = optimset('PlotFcns',@optimplotfval);

Parameter_IC = [ 2.109863844471290   0.463715523994944   0.111067013285918   0.899102608874919   0.001982661666695   0.039527123628854   4.472204981869051   6.585838465052916   5.929574086728122   0.000077950504107   0.000693420866297   0.000066485731003   0.000006834430619   0.000132470998682   0.000006191068627   0.038829818396668   0.570664841298183  -0.026598214155149];
							   
Parameter_IC'

% keep all gains nonnegative, but no constraint on boas terms
Lower_Bound = [-1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1 -1 -1 -1 -1 -1];

Lower_Bound'

result = fmincon('tn_opt_llw_494_survey', Parameter_IC, [], [], [], [], Lower_Bound, [], [], options)

% name for figures
figsname = "llw_sim_017_sentry494_survey_only_18_param_const_bias"

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

