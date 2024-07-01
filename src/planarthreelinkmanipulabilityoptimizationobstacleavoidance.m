% based on A Constraints-based Method of the Inverse Kinematics for Redundant Manipulators
%%
clc
close all
clear all
l1=3;
l2=2.5;
l3=2.0;  
A = [];
b = [];
Aeq = [];   
beq = [];
 lb=[-1.95,-1.95,-1.95];
 ub=[1.95,1.95,1.95];
% x0 = (lb + ub)/2;
x0 = [-1.521;1.951;1.353];
R=3;

%position function f(theta) forward kinematics
%  nonlcon = @distcon;
%   [x,fval] = fmincon(@myObjective,x0,A,b,Aeq,beq,lb,ub,@distcon)
step=0.01
   for t=0:step:1
%   for t=0.15
   [x,fval,history,searchdir] = runfmincon(t); % passed t as global variable, 
   %there are 3 methods 1, nested fn 2, anonymous 3,global
   x
   theta_array=history.x; %stores output from  optimization at each iteration
%% working plot for single configuration
hold on
  th1_1=x(1)
  th2_1=x(2)
  th3_1=x(3)
%  mu_main=sqrt(-fval)
        x_1=l1*cos(th1_1)+l2*cos(th1_1+th2_1)+l3*cos(th1_1+th2_1+th3_1);
        y_1=l1*sin(th1_1)+l2*sin(th1_1+th2_1)+l3*sin(th1_1+th2_1+th3_1);
        plot(0,0, 'ko','MarkerFaceColor','k','MarkerSize', 8)
        hold on
        plot([0, l1*cos(th1_1),l1*cos(th1_1)+l2*cos(th1_1+th2_1),x_1],[0, l1*sin(th1_1),l1*sin(th1_1)+l2*sin(th1_1+th2_1),y_1],'k-o',...
            'linewidth',1.5,'MarkerFaceColor','r',...
            'MarkerSize',5) % plot pose manipulator configuration
        hold on 
         syms t1 real
    xt1=-1*cos(2*pi*t1)+3;
    yt1=-1*sin(2*pi*t1);
    fplot(xt1,yt1,[0,1])
    hold on 
    
    %plot obstacle
circle(4.3,-3.0,sqrt(2))
hold on
drawnow
pause(0.01)

   end

%%
 function f = myObjective(x)
 l1=3;
l2=2.5;
l3=2.0;
% note here: the manipulability from JJtranspose is explicitly expanded and
% given as objective function
f=-(0.5*l1^2*l2^2+(l1^2*l2^2)*l3^2+l3^2*l2*l1*cos(x(2))-0.5*l1^2*l2^2*cos(2*x(2))+l1*l3*l2^2*cos(x(3))-...
    l2^2*l3^2*cos(2*x(3))-l1^2*l3^2*cos(2*x(2)+2*x(3))-l1^2*l2*l3*cos(2*x(2)+x(3))-l1*l2*l3^2*cos(x(2)+2*x(3)));
 end
 
 function [c,ceq] = distcon(x)
 %your nonlinear constraint function needs to be a function of one input variable
 l1=3;
l2=2.5;
l3=2.0;
 R=3;
   t=0.8;
 f1=[l1*cos(x(1))+l2*cos(x(1)+x(2))+l3*cos(x(1)+x(2)+x(3));l1*sin(x(1))+l2*sin(x(1)+x(2))+l3*sin(x(1)+x(2)+x(3))]
%  c = norm(f1)-R;
%  c=[]; % without inequality
  c=-(l1*cos(x(1))+l2*cos(x(1)+x(2))-4.3)^2-(l1*sin(x(1))+l2*sin(x(1)+x(2))+3)^2+(sqrt(2))^2; % with obstacle inequality
 step=0.02;
%  for t=0:step:1
 X=[-1*cos(2*pi*t)+3;-1*sin(2*pi*t)];
 ceq = X-f1;
%  end
end