%  2018-10-04 LLW new top level format for optimization to (a) allow
%  optimization in segments of a dive file specified by times, and (b)
%  to pass on estimator state between segments, (c) this will enable
%  use of different gains in course and fine aligmnent segments

% declare tn global
global tn;

% set optimization options
tn.options = optimset('PlotFcns',@optimplotfval,'MaxFunEvals',30000);

tn.TrueNorth_Package_dir = '/home/llw/kvh_catkin_ws/src/truenorth'

% exp dir with subdirectories /kvh, /phine, and /proc
tn.EXP_dir = '/home/llw/llw/sentry_2018/data/2018-sentry-gyro/dives_10hz/sentry494'

% name of full dive file name expt file with no suffix such as 2018_08_21_12_45'
% we will parse segmends from this file with tslice
tn.dive_file_name  = 'dive_bottom'


tn.sim_name = 'Sentry_492_Survey';

% Define segment time boundaries.
% Segments are contiguous, end of one is beginning of next
tn.segment_times = [ 2018 08 27 02 00 00;...
		     2018 08 27 02 30 00;...
		     2018 08 27 03 00 00];
%		     2018 08 27 08 30 00];

% name the time boundaries
tn.segment_time_names = ["begin_survey",...
			 "surveying   ",...
			 "end_survey  "];

% name the segments
tn.segment_names = ["Coarse_Alignment",...
                    "Fine_Alignment"];

% count the segments
tn.segment_count = size(segment_times,1)-1;

% start with segment 1
tn.segment_num  = 1;

for i = 1:tn.segment_count
  tn.segment_times_unix = ymdhms_to_sec(tn.segment_times(i,1),tn.segment_times(i,2),tn.segment_times(i,3),tn.segment_times(i,4),tn.segment_times(i,6),tn.segment_times(i,6));

% make segment files with tslice

% call tn_opt_llw_segment to optimize

% sim the segment with optimized parameters to get final state of estimator

% save the final state of the estimator  
  
end 


