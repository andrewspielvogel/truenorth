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
data.ang = M(:,1:3);
data.acc = M(:,4:6);
data.mag = M(:,7:9);
data.status = M(:,10:15);
data.temp = M(:,16);
data.stamp = M(:,17);
data.t = M(:,18);
data.seq_num = M(:,19);

end

