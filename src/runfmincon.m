function [xsol,fval,history,searchdir] = runfmincon(t)
 
% Set up shared variables with outfun
history.x = [];
history.fval = [];
searchdir = [];
 
% Call optimization
% x0 = [-1 1];
x0 = [-1.521;1.951;1.353];
 lb=[-1.95,-1.95,-1.95];
 ub=[1.95,1.95,1.95];
 options = optimoptions(@fmincon,'OutputFcn',@outfun,... 
     'Display','iter','Algorithm','active-set');
% options = optimoptions(@fmincon,'OutputFcn',@outfun,... 
%     'Display','iter','Algorithm','sqp');
 [xsol,fval] = fmincon(@myObjective,x0,[],[],[],[],lb,ub,@distcon,options);

% [xsol,fval] =
% fmincon(@myObjective,x0,[],[],[],[],[],[],@distcon,options); % you forgot
% to give lb and ub, so solutions were out of bounds for certain timesteps
 
% function stop = outfun(x, optimValues, state)
% stop = false;
% hold on;
% plot(x(1),x(2),'.');
% drawnow
% end
 function stop = outfun(x,optimValues,state)
     stop = false;
 
     switch state
         case 'init'
             hold on
         case 'iter'
         % Concatenate current point and objective function
         % value with history. x must be a row vector.
           history.fval = [history.fval; optimValues.fval];
%            history.x = [history.x; x];
           history.x = [history.x x];
         % Concatenate current search direction with 
         % searchdir.
           searchdir = [searchdir;... 
                        optimValues.searchdirection'];
%            plot(x(1),x(2),'o');
         % Label points with iteration number and add title.
         % Add .15 to x(1) to separate label from plotted 'o'.
%            text(x(1)+.15,x(2),... 
%                 num2str(optimValues.iteration));
%            title('Sequence of Points Computed by fmincon');
         case 'done'
             hold off
         otherwise
     end
  end
 
 function f = myObjective(x)
l1=3;
l2=2.5;
l3=2.0;
f=-(0.5*l1^2*l2^2+(l1^2*l2^2)*l3^2+l3^2*l2*l1*cos(x(2))-0.5*l1^2*l2^2*cos(2*x(2))+l1*l3*l2^2*cos(x(3))-...
    l2^2*l3^2*cos(2*x(3))-l1^2*l3^2*cos(2*x(2)+2*x(3))-l1^2*l2*l3*cos(2*x(2)+x(3))-l1*l2*l3^2*cos(x(2)+2*x(3)));
 end
 
 function [c,ceq] = distcon(x)
 %your nonlinear constraint function needs to be a function of one input variable
 l1=3;
l2=2.5;
l3=2.0;
 R=3;
%     t=0.5
% t=0.2
 f1=[l1*cos(x(1))+l2*cos(x(1)+x(2))+l3*cos(x(1)+x(2)+x(3));l1*sin(x(1))+l2*sin(x(1)+x(2))+l3*sin(x(1)+x(2)+x(3))]
%  c = norm(f1)-R;
%   c=[]; % without inequality or obstacle avoidance
%for obstacle
    c=-(l1*cos(x(1))+l2*cos(x(1)+x(2))-4.3)^2-(l1*sin(x(1))+l2*sin(x(1)+x(2))+3)^2+(sqrt(2))^2; % with obstacle inequality
  %defining boundary 
   % Nonlinear inequality constraints
%    c1=-(x(2)-1.7)^2-(x(3)-1.3)^2+0.2^2;
%    c2=-(x(2)-1.64)^2-(x(3)-1.17)^2+0.1^2
%    c3=-(l1*cos(x(1))+l2*cos(x(1)+x(2))-4.3)^2-(l1*sin(x(1))+l2*sin(x(1)+x(2))+3)^2+(sqrt(2))^2; % with obstacle inequality
%   
% c = [c1;     
%      c2;
%      c3];
%  step=0.006;
%    for t=0.1:step:4 %cant add loop inside constraints
 X=[-1*cos(2*pi*t)+3;-1*sin(2*pi*t)];
 ceq = X-f1;
%    end
 end
end