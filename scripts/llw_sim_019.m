% 2018-10-03 LLW sentry494 first 30 min of survey portion only, params from llw_sim_013

% 2018-10-04 LLW picking up where the first SIM 019 timed out on 'MaxFunEvals' default value of 3000, now set to 30000

options = optimset('PlotFcns',@optimplotfval,'MaxFunEvals',30000);

% param IC for first sim 019 SECOND TRY now using 494 data
Parameter_IC = [ 2.109863844471290   0.463715523994944   0.111067013285918   0.899102608874919   0.001982661666695   0.039527123628854   4.472204981869051   6.585838465052916   5.929574086728122   0.000077950504107   0.000693420866297   0.000066485731003   0.000006834430619   0.000132470998682   0.000006191068627   0.038829818396668   0.570664841298183  -0.026598214155149];

% param IC fo second sim 019, which is the result of the first BAD sim 019
% Parameter_IC = [    1.208381523574572   1.103108281299558   0.388817244380786   2.343497024930285   0.170708351807067   0.171886136306531   6.788565619990310  19.975027210096204   3.117615296185519   0.006381508118805   0.194440852713081   0.006532618582973   0.119428707814936   0.111317663845153   0.028165229297016  -0.163186551390648   3.655647384003520   0.967574663835923 ];
							   
Parameter_IC'

% keep all gains nonnegative, but no constraint on boas terms
Lower_Bound = [-1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1 -1 -1 -1 -1 -1];

Lower_Bound'

result = fmincon('tn_opt_llw_494_survey_first_30', Parameter_IC, [], [], [], [], Lower_Bound, [], [], options)

% name for figures
figsname = "llw_sim_019_sentry494_survey_only_first_30_min_18_param_const_bias"

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
