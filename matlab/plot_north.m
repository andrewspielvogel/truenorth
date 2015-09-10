function [NED,w,deg] = plot_north(data,bias,N,num_samples_2_avg,graph_type)
% plot_north plots the NED frames and angular velocities 
% that are calculated from a specified number of samples
%
% Andrew Spielvogel
% andrewspielvogel@gmail.com
%
% July 2015
%


num_samples = size(data.ang);

samp = floor(num_samples(1)/num_samples_2_avg);

%plot north
if(graph_type>1)
    
    figure;
    grid
    xlabel('x');
    ylabel('y');
    zlabel('z');
    hold on;
    plotv(N','-c');
    
end

for i=1:samp

    % find w, NED for each averaged set of samples
    w(i,:) = mean(data.ang(num_samples_2_avg*(i-1)+1:num_samples_2_avg*i,1:3))-bias.ang;

    NED.d(i,:) = -(mean(data.acc(num_samples_2_avg*(i-1)+1:num_samples_2_avg*i,1:3))-bias.acc);
    NED.e(i,:) = cross(NED.d(i,:),w(i,:));
    NED.n(i,:) = cross(NED.e(i,:),NED.d(i,:));

    N          = N/norm(N);
    NED.n(i,:) = NED.n(i,:)/norm(NED.n(i,:));
    NED.e(i,:) = NED.e(i,:)/norm(NED.e(i,:));
    NED.d(i,:) = NED.d(i,:)/norm(NED.d(i,:));
    n          = NED.n(i,:);
    
    % find the angle of the calculated north from true north
    true = atan2(N(2),N(1));
    calc = atan2(n(2),n(1));
    rad  = true-calc;
    
    if(rad<-pi)
        rad = rad+2*pi;
    elseif(rad>pi)
        rad = rad-2*pi;    
    end

    deg(i) = rad*180/pi;
    
    
    % plot NED, w is turned on
    if (graph_type>1)

    plot3([0,n(1)],[0,n(2)],[0,n(3)],'-b');

    wn = w(i,:)/norm(w(i,:));
    plot3([0,wn(1)],[0,wn(2)],[0,wn(3)],'-k');

    dn = NED.d(i,:)/norm(NED.d(i,:));
    plot3([0,dn(1)],[0,dn(2)],[0,dn(3)],'-r');

    en = NED.e(i,:)/norm(NED.e(i,:));
    plot3([0,en(1)],[0,en(2)],[0,en(3)],'-g');

    end

end

% get mean and std of samples
n_mean = mean(deg);
n_std  = std(deg);

% plot hists
if (graph_type~=2&&graph_type>0)

    plot_comp(NED.n,50,'north');
    plot_comp(w,50,'w');
    figure;
    hist(deg,50);
    grid;
    xlab = sprintf('Deg Mean=%.3f, Std Dev=%.3f',n_mean, n_std);
    xlabel(xlab);

end

NED.R = [NED.n(end,:);NED.e(end,:);NED.d(end,:)]';


