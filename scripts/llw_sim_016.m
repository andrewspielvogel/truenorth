% 2018-09-13 LLW sentry494, use fmincon to specify lower bound on all gains is zero
% LLW Sim 015 24 param:  k_ang_bias and k_acc_bias vary, set IC to 1e-4 and 1e-3 % uses tn_opt_llw_sim015

		 % 2018-09-11 LLW Sim #014: 
% LLW Sim #14 - params from mean of sim 11 and sim 12
% uses tn_opt_llw_sim012

options = optimset('PlotFcns',@optimplotfval);

% 24 param ICs from sim012, 6 are guesses
Parameter_IC = [  1.687671092064885   0.469109803953303   0.010439153549702   1.138173595062082   0.000114226145155   0.000297953812362   6.424765609312459  12.622925306894928   5.019888453901515   0.000048947572261   0.001273222247219   0.000043934905607    1e-3 1e-3 1e-3   1e-4 1e-4 1e-4    0.000010575733905   0.000025706148526   0.000005241694846   0.041914958322302  -0.012877318762107   0.004120792504163 ]; 
							   
Parameter_IC'

% keep all gains nonnegative, but no constraint on boas terms
Lower_Bound = [-1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1 -1 -1 -1 -1 -1];

Lower_Bound'

result = fmincon('tn_opt_llw_sim016', Parameter_IC, [], [], [], [], Lower_Bound, [], [], options)

% name for figures
figsname = 'llw_sim_016_sentry492_24_param'

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

