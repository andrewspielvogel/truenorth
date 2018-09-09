% name for figures
figsname = "llw_sim_007"

% random string for figures
figs_random_string = random_string(10)

for fignum = 1:6
  % select figure
  figure(fignum)
  % construct filename
  filename = sprintf('%02d_%s_%s.pdf',fignum,figsname,figs_random_string)
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
