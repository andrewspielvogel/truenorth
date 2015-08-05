function deg = gyro_sensitivity(ang_vel,grav_vec,factors,samples_per,data_points,is_w,plot_fig)
%function for doing sensitivity analysis
%
% Andrew Spielvogel
% andrewspielvogel@gmail.com
%
% August 2015
%

% sampling freq
freq = 1000;

num_factors = size(factors);
num_runs = size(samples_per);
bias.ang = [0,0,0];
bias.acc = [0,0,0];

for j=1:num_runs(2)
    for i=1:num_factors(2)
    
        w_sig = 2/(10000*sqrt(1/freq));
        a_sig = .12*sqrt(3)/(1000*sqrt(1/freq));
        
        if (is_w)
            w_sig = factors(i)*w_sig; 
        else
            a_sig = factors(i)*a_sig;
        end
        
        samples = gen_samp(ang_vel,grav_vec,samples_per(j)*data_points,w_sig,a_sig);
        [n,w,deg{j}.d(i,:)] = plot_north(samples,bias,ang_vel(1:2),samples_per(j),0);
        deg{j}.mean(i) = mean(deg{j}.d(i,:));
        deg{j}.std(i)  = std(deg{j}.d(i,:));
    
    end

    if(plot_fig>1)
        figure
        hold on;
        for i=1:data_points
            plot(factors,deg{j}.d(:,i),'or');
        end
        plot(factors,mean(deg{j}.d,2),'-b');
        p = polyfit(factors,deg{j}.std,2);
        f=polyval(p,factors);
        plot(factors,f,'-k')
    end
end

if(plot_fig>0&&plot_fig~=2)
    
    colors = {'-xb','-*r','-^m','-sk','-dg'};
    figure;
    hold on;
    for i=1:num_runs(2)
        
        plot(factors,deg{i}.std,colors{i});        
        
        
    end
    
    if (is_w)
        title('w sensitivity'); 
    else
        title('a sensitivity'); 
    end
        
    ylabel('std (^o)');
    xlabel('noise multiplier');
    leg = cellstr(num2str(samples_per(:)/freq));
    legend(leg);
    
end