function R = rph2R(ang)

R = Rz(ang(1))*Ry(ang(2))*Rx(ang(3));

function R = Rx(x)

R = [1,0,0;0,cos(x),-sin(x);0,sin(x),cos(x)];

function R = Ry(y)

R = [cos(y),0,sin(y);0,1,0;-sin(y),0,cos(y)];

function R = Rz(z)

R = [cos(z),-sin(z),0;sin(z),cos(z),0;0,0,1];