function data = parse_csv( filename )
%parse csv for kvh_1775 gyro data
%
% Andrew Spielvogel
% andrewspielvogel@gmail.com
%
% July 2015
%


% read file
M = csvread(filename);

% create structure
data.ang = M(:,1:3)';
data.acc = M(:,4:6)';
data.mag = M(:,7:9)';
data.status = M(:,10:15);
data.temp = M(:,16);
data.stamp = M(:,17)';
data.seq_num = M(:,18);
data.hz = round(1/(((data.stamp(end)-data.stamp(1)))/size(data.stamp,2)));
data.acc_est = M(:,19:21)';
data.bias.ang = M(:,22:24)';
data.bias.acc = M(:,25:27)';
data.bias.z = M(:,28:30)';
data.att = M(:,31:33)';

end

