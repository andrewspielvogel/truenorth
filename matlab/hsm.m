function ans = hours(unix_time)
% Mar 23 2007 LLW Computes monotinic hours since midnight

ans = ssm_noclash(unix_time) * (1/3600);
