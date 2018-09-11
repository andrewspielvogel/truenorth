% 2018-09-11 LLW sentry494, use fmincon to specify lower bound on all gains is zero
% LLW Sim #11 - restart where LLW Sim #9 was interrupted

options = optimset('PlotFcns',@optimplotfval);

Parameter_IC = [  +1.417247755813,   +0.297761068581,   +0.001759845346 ...  % k_g
                  +0.954331509909,   +0.000016807223,   +0.000101301835 ...  % k_north
                  +5.042689230923,  +12.132402611122,   +5.390553149311 ...  % k_acc
                  +0.000057558063,   +0.000186782528,   +0.000047219658 ...  % k_E_n
                  +0.000004455125,   +0.000000886481,   +0.000010108353 ...  % ang_bias
                  +0.054166628608,   -0.150795608218,   -0.006101756988 ];   % acc_boas
							   
Parameter_IC'

% keep all gains nonnegative, but no constraint on boas terms
Lower_Bound = [-1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1 -1 -1 -1 -1 -1];

Lower_Bound'

result = fmincon('tn_opt_llw_sim011', Parameter_IC, [], [], [], [], Lower_Bound, [], [], options)

% name for figures
figsname = "llw_sim_009_18_param_const_bias"

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

