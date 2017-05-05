function out = parse_bag(input)
lat = 39.32*pi/180;
r = 6371*1000;

Ren = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];
a_e = [cos(lat);0;sin(lat)] - (15.04*pi/180/3600)^2*cos(lat)*[r;0;0]/9.81;
a_n = Ren'*a_e;

fileID = fopen('/home/spiels/log/data2.KVH','w');

bag = rosbag(input);
bag2 = select(bag,'Time',[bag.StartTime, bag.EndTime],'Topic','/truenorth/gyro_data');

end_t = 5;
hz = 100;

if end_t>((bag.EndTime-bag.StartTime)/60)
    end_t = (bag.EndTime-bag.StartTime)/60;
end

msg = readMessages(bag2,1:hz*60*end_t);

num = size(msg,1);

out.att = zeros(3,num);
out.ang = zeros(3,num);
out.acc = zeros(3,num);
out.mag = zeros(3,num);
out.bias.ang = zeros(3,num);
out.bias.acc = zeros(3,num);
out.phins_att = zeros(3,num);
out.acc_v = zeros(3,num);

out.t = 0:1/hz:end_t*60-1/hz;

for i=1:num
    
    out.att(:,i) = msg{i}.Att;
    out.bias.ang(:,i) = msg{i}.Bias.Ang;
    out.bias.acc(:,i) = msg{i}.Bias.Acc;
    out.phins_att(:,i) = msg{i}.Bias.Z;
    out.acc(:,i) = msg{i}.Kvh.Imu.Acc;
    out.ang(:,i) = msg{i}.Kvh.Imu.Ang;
    out.mag(:,i) = msg{i}.Kvh.Imu.Mag;
    
    
    

end

out.acc = my_lowpass(out.acc',hz,1,0.9)';
out.ang = my_lowpass(out.ang',hz,1,0.9)';


for i=1:num
    R = rph2R(fliplr(out.phins_att(:,i)'*pi/180)');

    out.acc_v(:,i) = out.acc(:,i) - R'*a_n;
    fprintf(fileID,'IMU_RAW, %.40f,%.40f,%.40f, %.35f,%.35f,%.35f,0,0,0, 0, 0, %.30f, 0,1,1,1,1,1,1,%f,%f,%f,%f,%f,%f,%f,%f,%f \n',out.ang(1,i),out.ang(2,i),out.ang(3,i),out.acc(1,i),out.acc(2,i),out.acc(3,i),out.t(i),R(1,1),R(1,2),R(1,3),R(2,1),R(2,2),R(2,3),R(3,1),R(3,2),R(3,3));
end