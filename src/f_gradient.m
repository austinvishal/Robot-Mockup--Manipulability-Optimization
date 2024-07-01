function [fd] = f_gradient(fun, x0, del)
% numerical gradient of function fun
% at point x0, for a delta of del

n = max(size(x0));
fd = [];

for k=1:n
    
    x_l = x0; 
    x_l(k) = x_l(k)-del;
    fx_l = fun(x_l);
    
    x_r = x0; 
    x_r(k) = x_r(k)+del;
    fx_r = fun(x_r);
    
    fd(k) = (fx_r-fx_l)/(2*del);
end

fd = fd';


