function f = plot_sensitivity(deg,setup)
%function for ploting sensitivity analysis
%
% Andrew Spielvogel
% andrewspielvogel@gmail.com
%
% August 2015
%

% initialize line colots
colors = {'-xb','-*r','-^m','-sk','-dg','-ob','-+r','--xm','-.<k',':*g'};

num_runs = size(setup.samples_per);

figure;
hold on;

% plot each run
for i=1:num_runs(2)
        
    plot(setup.factors,deg{i}.std,colors{i});        
        
end

% write title
if (setup.is_w)

    title('w sensitivity'); 

else
    
    title('a sensitivity');
    
end

% label graph
ylabel('std (^o)');
xlabel('noise multiplier');
leg = cellstr(strcat(num2str(setup.samples_per(:)/setup.freq),' Second Avg'));
legend(leg,'Location','best');
grid;
    


%     figure;
%     hold on;
%     for i=1:num_runs(2)
%         
%         p = polyfitZero(factors,deg{i}.std,2);   
%         val = polyval(p,factors);
%         plot(factors,val,colors{i});
%         
%     end