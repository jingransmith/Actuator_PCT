%% Solution of 2-D Nonlinear System
%%
clc; clear all; close all;
% This example shows how to solve two nonlinear equations in two variables.
% The equations are
%
% $$ \begin{array}{c}
% {e^{ - {e^{ - ({x_1} + {x_2})}}}} = {x_2}\left( {1 + x_1^2} \right)\\
% {x_1}\cos \left( {{x_2}} \right) + {x_2}\sin \left( {{x_1}} \right) = \frac{1}{2}.
% \end{array} $$
%%
% Convert the equations to the form $F(x) = \bf{0}$.
%
% $$\begin{array}{c}
% {e^{ - {e^{ - ({x_1} + {x_2})}}}} - {x_2}\left( {1 + x_1^2} \right) = 0\\
% {x_1}\cos \left( {{x_2}} \right) + {x_2}\sin \left( {{x_1}} \right)
% - \frac{1}{2} = 0. \end{array} $$
%%
% Write a function that computes the left-hand side of these two equations.
%
% <include>root2d.m</include>
%
% Save this code as a file named |root2d.m| on your MATLAB(R) path.

%%
i = 0; l = 0; dt = 0.05;

% Not Used Here
% halfsecsamp = floor(0.5/dt);

%perform in 2D instead of 3D

% I generated Random Number Generator between 0 and 1
rand_x = rand_num();
x = rand_x;

% Gravity
g=10.0; 

t=[0:dt:10];

%test piece length (seperates 2 acuators)
len=3;
bh=zeros(1,length(t));
bh(1) =len/2; %starts in the centre

%anglez = ((x*200)./100.0)*(pi/8.0)-(pi/8.0);
% hc = cos(angley).*cv;
% vc = sin(angley).*cv;
% y1 = ((x.*100-70.0)/50.0)*3;
% y2 = ((x.*100-30.0)/50.0)*3;
y1=sin(t)+1;
y2=sin(t+0.3)+1; %make all in pos range
angley1=zeros(1,length(t));
angley2=zeros(1,length(t));
%angley3=zeros(1,length(t)); %angley3=angle actuator 2 makes with base plate
%angley1(1) = acos((y1(1)-y2(1)*sin(angley3(1)))/len); %angles in radians
angley1_OG= acos((y1-y2)/len);
%angley2(1) =  1.5*pi-angley1(1)-angley3; %270 degrees total=1.5pi radian
angley2_OG =  1*pi-angley1_OG;
cv=zeros(1,length(t));
cv(1) = dt*g*cos(angley1(1)) ; %sensor inputs= ball velocity initial
cv_OG=dt*g*cos(angley1) ;

% Solve the system of equations starting at the point |[0,0]|.
fun = @(x)paramfunJS(x,len,y1(1),y2(1));
x0 = [0,0];
x = fsolve(fun,x0)

%% 
% Copyright 2012 The MathWorks, Inc.