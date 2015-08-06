function [deg, setup] = gyro_sensitivity(ang_vel,grav_vec,factors,samples_per,data_points,is_w,plot_fig)
%function for doing sensitivity analysis
%
% Andrew Spielvogel
% andrewspielvogel@gmail.com
%
% August 2015
%


% sampling freq
freq = 1000;

% store simulation settings
setup.factors = factors;
setup.samples_per = samples_per;
setup.data_points = data_points;
setup.ang_vel = ang_vel;
setup.grav_vec = grav_vec;
setup.is_w = is_w;
setup.freq = freq;

% initialize variables
num_factors = size(factors);
num_runs    = size(samples_per);

w_sig_ins = 2/(10000*sqrt(1/freq));
a_sig_ins = .12*sqrt(3)/(1000*sqrt(1/freq));

bias.ang = [0,0,0];
bias.acc = [0,0,0];


for j=1:num_runs(2)
    for i=1:num_factors(2)

        % determine whether sensitivity analysis is for a or w
        if (is_w)
            w_sig = factors(i)*w_sig_ins; 
            a_sig = a_sig_ins;
        else
            a_sig = factors(i)*a_sig_ins;
            w_sig = w_sig_ins;
        end
        
        % generate samples
        samples = gen_samp(ang_vel,grav_vec,samples_per(j)*data_points,w_sig,a_sig);
        
        % average over specified number of samps to get w, NED
        [~,~,deg{j}.d(i,:)] = plot_north(samples,bias,ang_vel(1:2),samples_per(j),0);
        
        % store the mean and std
        deg{j}.mean(i) = mean(deg{j}.d(i,:));
        deg{j}.std(i)  = std(deg{j}.d(i,:));
    
    end

    % if specified, plot mean, std, and points
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
        grid;
        
    end
end

% if specified, plot sensitivity graph
if(plot_fig>0&&plot_fig~=2)
    
    plot_sensitivity(deg,setup);
    
end