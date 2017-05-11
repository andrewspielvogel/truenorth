function out = plot_post_process( data )
    num_cat=10;
    num_samples = (size(data,1)/num_cat);
    data_parsed = zeros(num_cat,num_samples);
    for i=1:num_cat

        data_parsed(i,:) = data((i-1)*num_samples+1:i*num_samples);
        
    end

out.t = data_parsed(1,:);
out.stamp = out.t;
out.att = data_parsed(2:4,:);
out.Rb_rph = data_parsed(5:7,:);
out.east_n = data_parsed(8:10,:);


% figure;
% plot(data_parsed(1,:),data_parsed(2:4,:)*180/pi);
% grid on;
% 
% figure;
% plot(data_parsed(1,:),data_parsed(5:7,:)*180/pi);
% grid on;