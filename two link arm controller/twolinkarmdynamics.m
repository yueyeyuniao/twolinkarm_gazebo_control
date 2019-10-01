function [H,C,B] = twolinkarmdynamics(q,qd)
      % robot configuration
      l1 = 1; l2 = 1;  
      m1 = 1; m2 = 1;  
      g = 9.81;
      b1=.1;  b2=.1;
      I1= m1*l1^2; 
      I2= m2*l2^2;
      
      % keep it readable:
      m2l1l2 = m2*l1*l2;  % occurs often!

      c = cos(q(1:2,:));  s = sin(q(1:2,:));  s12 = sin(q(1,:)+q(2,:));
      
      h12 = I2 + m2l1l2*c(2);
      H = [ I1 + I2 + m2*l1^2 + 2*m2l1l2*c(2), h12; h12, I2 ];
      
      C = [ -2*m2l1l2*s(2)*qd(2), -m2l1l2*s(2)*qd(2); m2l1l2*s(2)*qd(1), 0 ];
      G = g*[ m1*l1*s(1) + m2*(l1*s(1)+l2*s12); m2*l2*s12 ];
            
      % accumate total C and add a damping term:
      C = C*qd + G + [b1;b2].*qd;

      B = eye(2);
    end