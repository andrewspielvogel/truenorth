function out = plot_E(samp,hz)
dt = 1/hz;

ang = samp.ang;%my_lowpass(samp.ang',hz,1,10)';
acc = samp.acc;%my_lowpass(samp.acc',hz,1,10)';

num = size(samp.t,2);
East = zeros(3,num);
tot = zeros(3,num);
a_dot = zeros(3,num);
for i=2:num
   
    tot(:,i) = skew(ang(:,i))*acc(:,i);
    a_dot(:,i) = (acc(:,i)-acc(:,i-1))/dt;
    East(:,i) = skew(ang(:,i))*acc(:,i)+a_dot(:,i);
    %East(:,i) = East(:,i)/norm(East(:,i));
    
end
out.East = East;
out.a_dot = a_dot;
out.tot = tot;
figure;plot(samp.t,samp.ang,samp.t,ang);grid on;
figure;plot(samp.t,samp.acc,samp.t,acc);grid on;
figure;plot3(tot(1,:),tot(2,:),tot(3,:),'.');grid on;axis equal;xlabel('x');ylabel('y');zlabel('z');
figure;plot3(a_dot(1,:),a_dot(2,:),a_dot(3,:),'.');grid on;axis equal;xlabel('x');ylabel('y');zlabel('z');
figure;plot3(East(1,5:end),East(2,5:end),East(3,5:end),'.');grid on;axis equal;xlabel('x');ylabel('y');zlabel('z');title('measured east');
%figure;plot3(samp.E(1,:),samp.E(2,:),samp.E(3,:));grid on; axis equal;xlabel('x');ylabel('y');zlabel('z');title('true east');
%figure;plot3(samp.D(1,:),samp.D(2,:),samp.D(3,:),'.');grid on;axis equal;xlabel('x');ylabel('y');zlabel('z');title('true down');
%figure;plot3(samp.acc(1,:),samp.acc(2,:),samp.acc(3,:),'.');grid on;axis equal;xlabel('x');ylabel('y');zlabel('z');title('measured down');
