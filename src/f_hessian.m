function [fd] = f_hessian(fun, x0, del)
% numerical Hessian of function fun
% at point x0, for a delta of del

n = max(size(x0));
fd = [];

for k=1:n
    
    x_l = x0; 
    x_l(k) = x_l(k)-del;
    fxd_l = f_gradient(fun, x_l, del);
    
    x_r = x0; 
    x_r(k) = x_r(k)+del;
    fxd_r = f_gradient(fun, x_r, del);
    
    fd(k,:) = ( (fxd_r-fxd_l)/(2*del) )';
end

fd = fd';


