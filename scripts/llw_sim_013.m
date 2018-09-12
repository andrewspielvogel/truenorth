% 2018-09-12 LLW sentry494, use fmincon to specify lower bound on all gains is zero
% LLW Sim #13 - params from mean of sim 11 and sim 12
% uses tn_opt_llw_sim011

options = optimset('PlotFcns',@optimplotfval);

Parameter_IC = [  1.687671092064885   0.469109803953303   0.010439153549702   1.138173595062082   0.000114226145155   0.000297953812362   6.424765609312459  12.622925306894928   5.019888453901515   0.000048947572261   0.001273222247219   0.000043934905607   0.000010575733905   0.000025706148526   0.000005241694846   0.041914958322302  -0.012877318762107   0.004120792504163 ]; 
							   
Parameter_IC'
% keep all gains nonnegative, but no constraint on boas terms
Lower_Bound = [-1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1 -1 -1 -1 -1 -1];

Lower_Bound'

result = fmincon('tn_opt_llw_sim011', Parameter_IC, [], [], [], [], Lower_Bound, [], [], options)

% name for figures
figsname = "llw_sim_013_sentry494_18_param_const_bias"

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

