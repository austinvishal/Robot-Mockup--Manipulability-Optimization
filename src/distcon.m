function [c,ceq] = distcon(x,R,l1,l2,l3)
f=[l1*cos(x(1))+l2*cos(x(1)+x(2))+l3*cos(x(1)+x(2)+x(3));l1*sin(x(1))+l2*sin(x(1)+x(2))+l3*sin(x(1)+x(2)+x(3))]
 c = norm(f)-R;
 ceq = [];
end