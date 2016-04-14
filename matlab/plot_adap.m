function f = plot_adap(out,name)

num = size(out,2);

comp = ['x','y','z'];

figure
for i=1:3
   
    subplot(3,1,i);
    hold on;
    
    for j=1:num

        plot(out(j).t,out(j).bias.acc(i,:)-repmat(out(j).true.bias.acc(i),1,size(out(j).t,2)));
        
    end
    xlabel('time [s]');
    str = sprintf('bacc_%s error [g]',comp(i));
    ylabel(str);
    grid on;
end

if nargin>1
    legend(name);
end


figure
for i=1:3
   
    subplot(3,1,i);
    hold on;
    
    for j=1:num

        plot(out(j).t,out(j).bias.ang(i,:)-repmat(out(j).true.bias.ang(i),1,size(out(j).t,2)));
        
    end
    xlabel('time [s]');
    str = sprintf('bang_%s error [rad/s]',comp(i));
    ylabel(str);
    grid on;
end
if nargin>1
    legend(name);
end

figure
for i=1:3
   
    subplot(3,1,i);
    hold on;
    
    for j=1:num
        
        z = cross(out(j).true.bias.ang,out(j).true.bias.acc);
        plot(out(j).t,out(j).bias.z(i,:)-repmat(z(i),1,size(out(j).t,2)));
        
    end
    xlabel('time [s]');
    str = sprintf('bz_%s error',comp(i));
    ylabel(str);
    grid on;
end
if nargin>1
    legend(name);
end

figure;
for i=1:3
   
    subplot(3,1,i);
    hold on;
    
    for j=1:num
        
        plot(out(j).t,out(j).acc(i,:)-out(j).true.acc(i,:))
        
    end
    xlabel('time [s]');
    str = sprintf('acc_%s error [g]',comp(i));
    ylabel(str);
    grid on;
end
if nargin>1
    legend(name);
end

