% 2018-10-09 LLW sentry492 survey portion only, 18 params w/constant
% bias, param ICs from llw_sim_018, with RMS computed for entire
% interval and estimator initialized to Phins attitude.

options = optimset('PlotFcns',@optimplotfval);

% param IC from results of llw_sim_018
Parameter_IC = [   2.654854069185680   0.050877606998711   0.019530260704434   0.528780275557932   0.000393096861345   0.007796989428158   2.933228806739161   5.279262914982921   11.573582936502582   0.000000004737510   0.000302699700731   0.000062256563972   0.001172635119063   0.000277028980770   0.000005617260288   0.354013771721498    0.811718446490420  -0.130785794578031	];

Parameter_IC'

% keep all gains nonnegative, but no constraint on boas terms
Lower_Bound = [-1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1 -1 -1 -1 -1 -1];

Lower_Bound'

result = fmincon('tn_opt_llw_492_survey', Parameter_IC, [], [], [], [], Lower_Bound, [], [], options)

% name for figures
figsname = "llw_sim_022_sentry492_survey_only_18_param_const_bias"

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
