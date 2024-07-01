function [c,ceq] = collisionobstacle(l1,th1,l2,th2)
% c = x(1)^2 + x(2)^2 - 1;
c = -(l1*cos(th1)+l2*cos(th1+th2)-4.3)^2-(l1*sin(th1)+l2*sin(th1+th2)+3)^2+(sqrt(2))^2;
ceq = [];
end