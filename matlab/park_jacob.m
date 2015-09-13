function A = park_jacob(q)
  
  normq = norm(q);

  Q = skew(q);
  
  a = 1 - cos(normq);
  
  b = normq - sin(normq);
  
  A0 = a/normq^2 *skew(q);
  A1 = (b/normq^3) * (skew(q))^2;
  
  A = eye(3) - A0 + A1;
  
  if (norm(q)<1/1000000)
      
      A = eye(3);
      
  end
  
  
  
