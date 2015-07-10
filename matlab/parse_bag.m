function data = parse_bag(filepath)

bag = rosbag(filepath);
msgs = readMessages(bag);

num_msgs = bag.NumMessages;

for i=1:num_msgs
    
    data.temp(i,1)     = msgs{i}.Temp;
    data.seq_num(i,1)  = msgs{i}.SeqNum;
    data.stamp(i,:)  = [msgs{i}.Stamp.Sec; msgs{i}.Stamp.Nsec];
    data.t(i,1)        = msgs{i}.T;
    data.ang(i,:)    = msgs{i}.Ang;
    data.acc(i,:)    = msgs{i}.Acc;
    data.mag(i,:)    = msgs{i}.Mag;
    data.status(i,:) = msgs{i}.Status;
    
end