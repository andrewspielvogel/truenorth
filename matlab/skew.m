function m = skew(w)
% return skew of vector
%
% Andrew Spielvogel
% andrewspielvogel@gmail.com
%
% July 2015
%

m = [0,-w(3),w(2);
     w(3),0,-w(1);
     -w(2),w(1),0];
end
