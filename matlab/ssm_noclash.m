function ans = ssm_noclash(unix_time)
% Mar 23 2007 LLW Computes monotinic secs since midnight

[yr,mo,da,ho,mi,se] = sec_to_ymdhms(unix_time);
ans = (unix_time - ymdhms_to_sec(yr,mo,da,0,0,0));
