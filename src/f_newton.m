function [xs, res_vec] = f_newton(fun, a, b, n)
x = (a+b)/2;
% numerical gradient and Hessian
del = 0.01;
res_vec = [];
% 1-n iterations
for k=1:n
    
    res_vec = [res_vec; x];
    
    fd_x = f_gradient(fun, x, del);
    fdd_x = f_hessian(fun, x, del);
    
    x = x - fd_x/fdd_x;
    
end
xs = x;