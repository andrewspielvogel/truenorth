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

axis_fontsize = 13;
legend_fontsize = 12;
plotscale = 1.2;

num_runs = size(setup.samples_per);

fig_width = 700;
fig_height = 500;

figure('position',[10,10,fig_width,fig_height]);
hold on;

% plot each run
for i=1:num_runs(2)
        
    plot(setup.factors,deg{i}.std,colors{i});        
        
end

% write title
if (setup.is_w)

    title('Angular Velocity Sensitivity','FontSize',axis_fontsize); 

else
    
    title('Acceleration Sensitivity','FontSize',axis_fontsize);
    
end

% label graph
ylabel('Std [degrees]','FontSize',axis_fontsize);
xlabel('Noise Multiplier','FontSize',axis_fontsize);
legend_str = cellstr(strcat(num2str(setup.samples_per(:)/setup.freq),' Sec Avg'));
lhand = legend(legend_str,'Orientation','horizontal','FontSize',legend_fontsize);
grid;

% reduce the lenght of the X-axis (by 10%)
pos = get(gca,'Position');
pos(2) = plotscale*pos(2);
set(gca,'Position',pos);

pos = get(lhand,'Position');
pos(1:2) = [(1-pos(3))/2,.01];
set(lhand,'Position',pos);


figure('position',[10,10,fig_width,fig_height]);
hold on;

skip = 20;
% plot each run
for i=1:num_runs(2)
        
    %plot(setup.factors,deg{i}.mean,colors{i});      
    errorbar(setup.factors(1:skip:end),deg{i}.mean(1:skip:end),deg{i}.std(1:skip:end),colors{i});      
        
end

% write title
if (setup.is_w)

    title('Angular Velocity Sensitivity','FontSize',axis_fontsize); 

else
    
    title('Acceleration Sensitivity','FontSize',axis_fontsize);
    
end

% label graph
ylabel('Mean [degrees])','FontSize',axis_fontsize);
xlabel('Noise Multiplier','FontSize',axis_fontsize);
legend_str = cellstr(strcat(num2str(setup.samples_per(:)/setup.freq),' Sec Avg'));
lhand = legend(legend_str,'Orientation','horizontal','FontSize',legend_fontsize);
grid;

% reduce the lenght of the X-axis (by 10%)
pos = get(gca,'Position');
pos(2) = plotscale*pos(2);
set(gca,'Position',pos);

pos = get(lhand,'Position');
pos(1:2) = [(1-pos(3))/2,.01];
set(lhand,'Position',pos);
    


%     figure;
%     hold on;
%     for i=1:num_runs(2)
%         
%         p = polyfitZero(factors,deg{i}.std,2);   
%         val = polyval(p,factors);
%         plot(factors,val,colors{i});
%         
%     end