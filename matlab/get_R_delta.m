function Rs = get_R_delta(w_m,t)


S.w=w_m;
S.t=t;
R0 = eye(3);

[ts, Rs] = ode45(@R_delta_ode, t, R0, [], S);





function dR = R_delta_ode(t, R, S)

w_est = zeros(3,1);
w_est(1) = interp1(S.t,S.w(:,1),t);
w_est(2) = interp1(S.t,S.w(:,2),t);
w_est(3) = interp1(S.t,S.w(:,3),t);
R = reshape(R,[3,3]);
dR = R*skew(w_est);
dR = reshape(dR,[9,1]);

