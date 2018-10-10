function [err KVH PHINS CSV CONFIG PDF EXP_dir TrueNorth_Package_dir] = llw_sim_dive_survey_492_once(p) 

% if a phins data structure has been previously loaded, delete it  
global phins;
clear phins;

[err KVH PHINS CSV CONFIG PDF EXP_dir TrueNorth_Package_dir] = tn_opt_llw_492_survey(p);

% name for figures
figsname = 'llw_sim_dive_survey_492_survey';

% random string for figures
figs_random_string = random_string(10)

%  call andrew's python plotting file
filename = sprintf('%02d_%s_%s.pdf',0,figsname,figs_random_string);

python_plot_cmd = [TrueNorth_Package_dir '/python/plot_att.py -i ', CSV, ' -o ', filename, ' -p ' PHINS]
unix(python_plot_cmd);

% synoptic plots - may be redundant
for fignum = 1:5
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

cascade;
