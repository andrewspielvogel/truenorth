function out = int_plant(samp,DVL,bias)

if nargin < 3
   
    bias.ang = [0,0,0];
    bias.acc = [0,0,0];
    
end

if (any(strcmp('vel',fieldnames(samp)))&&DVL)
    disp('Using DVL corrections');
else
    disp('No DVL corrections');
end

% number of samples
num_samp = size(samp.acc,2);

% adjust t so that it starts at t0=0
t = samp.stamp-samp.stamp(1);

% allocate space
b_ang = zeros(3,num_samp);
acc   = zeros(3,num_samp);
b_acc = zeros(3,num_samp);
b_z   = zeros(3,num_samp);

% initial measurement
acc(:,1) = samp.acc(:,1);
b_ang(:,1) = bias.ang';
b_acc(:,1) = bias.acc';
b_z(:,1) = cross(bias.ang,bias.acc);


k1 = 1; % acc gain
k2 = .001;%.5; % b_acc gain
k2b = .001;
k3 = .001;%.0005; % b_ang gain
k3b = 0.001;
k4 = .000001; % b_z gain

for i=2:num_samp
    
    % define da and dt
    dt = 1/samp.hz;%t(i)-t(i-1);
    
    if (any(strcmp('vel',fieldnames(samp)))&&DVL)
    
        if floor(t(i))>0
            dvl_dt = 1;
            aveh = (samp.vel(:,floor(t(i))+1)-samp.vel(:,floor(t(i))))/dvl_dt;
        else
            aveh = zeros(3,1);
        end
        
        acc_cor = samp.acc(:,i-1) - aveh;
   
        da = acc(:,i-1) - acc_cor;
        
        % define dx terms
        da_est = skew(-samp.ang(:,i-1))*acc(:,i-1) + skew(samp.ang(:,i-1))*b_acc(:,i-1) + skew(-acc_cor)*b_ang(:,i-1) - b_z(:,i-1) - k1*da;
        dab    =  k2*skew(samp.ang(:,i-1))*da;
        dwb    = -k3*skew(acc_cor)*da;
        dzb    =  k4*da;
    
    else

         da = acc(:,i-1) - samp.acc(:,i-1);
        
        % define dx terms
        da_est = skew(-samp.ang(:,i-1))*acc(:,i-1) + skew(samp.ang(:,i-1))*b_acc(:,i-1) + skew(-samp.acc(:,i-1))*b_ang(:,i-1) - b_z(:,i-1) - k1*da;

        dab    =  k2*skew(samp.ang(:,i-1))*da;
        dwb    = -k3*skew(samp.acc(:,i-1))*da;
        dzb    =  k4*da;
        if i>60*samp.hz*5
            
            dab = k2b*skew(samp.ang(:,i-1))*da;
            dwb = -k3b*skew(samp.acc(:,i-1))*da;
            
        end
        
        
    end
    
    % integrate
    acc(:,i)   = acc(:,i-1)   + dt*da_est;
    b_acc(:,i) = b_acc(:,i-1) + dt*dab;
    b_ang(:,i) = b_ang(:,i-1) + dt*dwb;
    b_z(:,i)   = b_z(:,i-1)   + dt*dzb;
    
end

% save outputs
out.k = [k1;k2;k3;k4];
out.acc = acc;
out.bias.acc = b_acc;
out.bias.ang = b_ang;
out.bias.z = b_z;
out.t = t;
out.hz = samp.hz;
% 
% if (any(strcmp('true',fieldnames(samp)))||any(strcmp('noise',fieldnames(samp))))
% 
%     out.true = samp.true;
%     out.noise = samp.noise;
% 
% end
