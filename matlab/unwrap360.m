function ans = unwrap360( x )

% first map heading into the interval [0 360]
while(min(x)<0)
x = mod(x+360.0,360);
end;

% now scale to an even power of 2 so we can use unwrapn
x = x * ((2^16)/360.0);

%now unwrap it
x = unwrapn(x,16);

%now scale back to degrees
ans = x * (360.0/(2^16));
 
 
