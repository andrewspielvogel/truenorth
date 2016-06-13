function ans = dsm(unix_time)
% Mar 23 2007 LLW Computes monotinic days since midnight

ans = ssm_noclash(unix_time) * (1/(24*3600));
