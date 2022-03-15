%% Rick's Code for TG 17-07-2021 

clc; close all; clear all

%% Variables
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
bh_OG=zeros(1,length(t));
bh(1) =len/2; %starts in the centre
bh_OG(1)=bh(1);

y1_OG=sin(t+0.3)+5;
y2_OG=sin(t)+5; %make all in real length of actators
y1=zeros(1,length(t));y2=zeros(1,length(t));
y1(1)=y1_OG(1); y2(1)=y2_OG(1);
%figure;hold on; plot(y1_OG); plot(y2_OG); legend('y1_OG','y2_OG'); hold off;
angley1=zeros(1,length(t));
angley2=zeros(1,length(t));
angle_two=zeros(1,length(t)); %internal angle, should be continuous
angley3=zeros(1,length(t)); %angley3=angle actuator 2 makes with base plate

%try simultaneous equation approach, not linear equations for angles
% Solve the system of equations starting at the point |[0,0]|.
for n=1:length(t)
    if y1_OG(n)>y2_OG(n)
        fun = @(x)paramfunJS(x,len,y1_OG(n),y2_OG(n));
    else %y2>=y1
        fun = @(x)paramfunJS2(x,len,y1_OG(n),y2_OG(n));
    end
    x0 = [0,0];
    x = fsolve(fun,x0);
    if y1_OG(n)>y2_OG(n) %should still end up as cont func.
        %angley1(n)=x(1)+pi/2;  %mtrig manipulation to put in right quadrant
    else %y2>=y1
        %angley1(n)=pi/2-x(1);  %mtrig manipulation to put in right quadrant???
    end
    angley1(n)=-x(1);
    angley3(n)=x(2);
        %be sure to update in while loop code as well
    angle_two(n)=3/2*pi-angley1(n)-angley3(n);%plus negative
    if y1_OG(n)>y2_OG(n)
        angley2(n)=angle_two(n)-pi/2;
    else %y2>y1
        angley2(n)=angley1(n)-pi/2;
    end
end
figure;plot(angley1);hold on;plot(angley2);plot(angle_two);plot(angley3);legend('angle1','angle2','angle two','angley3')
figure; plot(angley1+angle_two+angley3);%should be constant


    %%

cv=zeros(1,length(t));
%cv(1) = dt*g*cos(angley1(1)) ; %sensor inputs= ball velocity initial
cv_OG=dt*g*cos(angley1) ;

%correct cv to be alogn x-axis only
for n=1:length(t)
    if y1(n)>y2(n)
        cv_OG(n)=cv_OG(n)*sin(angley1(n));
    else %y2>y1
        cv_OG(n)=cv_OG(n)*cos(angley1(n)-pi/2);
    end
end
cv(1)=cv_OG(1);

for n=1:length(t)
    bh_OG(n+1)=bh_OG(n) + cv_OG(n)*dt; % was *dt
end
%figure;plot(bh_OG);hold on;plot(cv_OG); legend('bh_OG','cv_OG');

ref1= 0.0185; ref2 = 0.0185; up = 0; vp = 0; ldp = 0; 
fvel = zeros(1,length(t)); facc = zeros(1,length(t)); lvel = zeros(1,length(t)); lacc = 0; 
for n=1:(length(t)-1)
    fvel_OG(n) = (y1_OG(n+1)-y1_OG(n))/dt;
    lvel_OG(n) = (y2_OG(n+1)-y2_OG(n))/dt;
end
fvel(1)=fvel_OG(1);lvel(1)=lvel_OG(1);
fgain = 300.0; lgain=300.0; fslow=500; lslow = 500.0; %lslow 1000

% 3 x 4 Matrix of zeros
delay = zeros(3,4);
% % delay = [0,0,0,0,0;
% %          0,0,0,0,0;
% %          0,0,0,0,0];
drag = 0;
d = 1; % For Loop to iterate indefinitely
xx = 0;

ld=zeros(1,length(t)); v=zeros(1,length(t));
ld(1)= bh(1)-(5-y1(1)); ldp=ld(1);
%v(1)=len-bh(1)-(5-y2(1)*sin(angley3(1))); vp=v(1);
v(1)=bh(1)-(5-y2(1)*cos(angley3(1))); vp=v(1); %was sin
p1=zeros(1,length(t)); p2=zeros(1,length(t));

%% replace once off pulse generation
y1_OG(2:end)=zeros(1,length(y1_OG)-1);
y2_OG(2:end)=zeros(1,length(y2_OG)-1);
%%
%new PCT initialised
ref_xball=len/2; e1=zeros(1,length(t));
ref_vball=zeros(1,length(t)); K1=2000;
e2=zeros(1,length(t));
K2=2000;
ref_ld=zeros(1,length(t)); ref_v=zeros(1,length(t));

while (d > 0) 
    for t=2:length(y1) %was 2, could be 3?
            xx = xx + 1;
            if t==57
                d=1;%continue still but break pt
            end
            
            %% Solve for ball
            % Iterating over bh, bv
            %displacements
            bh(t) = bh(t-1) + cv(t-1)*dt; % dt,hc are constants
            %new PCT
            e1(t)=ref_xball-bh(t);
            ref_vball(t)=e1(t)*K1;

            % Iterating over velocities going backwards
            %cv(t+1) = cv(t) - (g*dt);     %g constant will need to change
            %new interum step for cv
            a_mult_t=dt*g*cos(angley1(t-1));
            if y1(t)>y2(t)
                cv(t) = cv(t-1)+a_mult_t*sin(angley1(t));
            else %y2>y1
                cv(t) = cv(t-1)+a_mult_t*cos(angley1(n)-pi/2);
            end
            %new PCT
            e2(t)=ref_vball(t)-cv(t);
%             syms angley1
%              %cv=dt*...*cos(angley1)*{sin or cos fn angley1}
%             expr= e2(t)*K2;
%             ref_ld(t)=int(expr);
            
            %% Establish relationships, relative (upward) displacement of acutaor
            % Id is retina1, relate ball and actuator 1
            ld(t) = bh(t)+(5-y1(t-1)); %*cos(angley1(t)) %was minus

            % v is retina2, relate ball and actuator 2
            %v(t) = len-bh(t)-(5-y2(t-1)*sin(angley3(t-1)));
            v(t) = bh(t)+(5-y2(t-1)*cos(angley3(t-1))); %was sin
            
            p1(t) = v(t)-vp;
            p2(t) = ld(t)-ldp;
            vp = v(t);
            ldp = ld(t);
            
            %
            %% Solve for actuators
            % Matlab Cannot Iterate over 0 index, hence 1-4 with Minus 1
            for j = 1:1:4
                delay(2,j+1) = delay(2,j);
                delay(3,j+1) = delay(3,j);
            end

            noisy = noise();
            delay(2,1) = p1(t) + noisy(1)/30000.0; %delays relat ld & v to velocities later. 
            delay(3,1) = p2(t) + noisy(1)/30000.0;

            %fvel(t) = fvel(t-1) + (fgain * (ref1-delay(1,1))-fvel(t-1))/fslow; %actuator 1 velocity -> relate to ld
            %lvel(t) = lvel(t-1) + (lgain * (ref2-delay(1,1))-lvel(t-1))/lslow; %actuator 2 verlocity -> relate to v
            fvel(t) = fvel(t-1) + (fgain * (ref1-delay(2,1))-fvel(t-1))/fslow; 
            lvel(t) = lvel(t-1) + (lgain * (ref2-delay(3,1))-lvel(t-1))/lslow; %was minus
%             lvel(t+1) = lvel(t) + (lgain * (ref2-delay(1,1))-lvel(t))/lslow;
%             e1=...
%             ref1=(fgain/fslow)*e1;
%             fvel(t+1)=fvel(t)+(fgain*(ld-len/2))/fslow;
            %how did get velocity from basically only references? 

        %     Thresholding
        %{
            if (lvel > 6.0)
                lvel = 6.0;
            end
            if (lvel < -6.0)
                lvel = -6.0;
            end
            if (fvel > 6.0)
                fvel = 6.0;
            end
            if (fvel < -6.0)
                fvel = -6.0;
            end
        %}  
        %
            y1(t) = y1(t-1) - fvel(t)*dt; %was plus, use adjusted speed? t+1? 
            y2(t) = y2(t-1) - lvel(t)*dt;
 
        %     runrate = sqrt((fx-fxp)*(fx-fxp)+(fz-fzp)*(fz-fzp));
            
            %update angles
            if y1(t)>y2(t)
                fun = @(x)paramfunJS(x,len,y1(t),y2(t));
            else
                fun = @(x)paramfunJS2(x,len,y1(t),y2(t));
            end
            x0 = [0,0];
            x = fsolve(fun,x0);
            if y1(t)>y2(t)
                angley1(t)=x(1)+pi/2;  %mtrig manipulation to put in right quadrant
            else %y2>=y1
                angley1(t)=pi/2-x(1);  %mtrig manipulation to put in right quadrant???
            end
            angley3(t)=x(2);
            %angley2(t+1)=1.5*pi-abs(x(1))-abs(x(2));
            if y1(t)>y2(t)
                angley2(t)=0.5*pi-angley1(t);
            else %y2>y1
                angley2(t)=pi-0.5*pi-(pi-angley1(t)); %internal angles of a right-angle-triangle
            end


            %% Stop the Code Execution
            %need to fix this to be constant for some period of time
            if (abs(cv(t)) < 0.3)     %relate to velocity instead, e.g. v=very small. 
                d = 0;
            end
            %can we make it ld & v converge?
    %}
            % Save Data for Maximum of 199 Points
            if (i < 500 )
                i = i + 1;
                retinaX(i) = ld(t);
                retinaY(i) = v(t);
                fielderX(i) = y1(t);
                fielderZ(i) = y2(t);
                ballX(i) = bh(t);
                NumOfDataPoints = i;
            end

    end
    break;
end

%actuator movement could, but does not necessarily have to, become
%synchronous. 
figure;
plot(fielderX,'-r','LineWidth',2)
hold on;
plot(fielderZ,'-b','LineWidth',2)
title('Actuator Vertical Distplacements');
xlabel('Iterations');
ylabel('vertical Y-Coords');
legend('Actuator1','Acturator2','Location','Best')

%retina views should converge at x=length/2
figure;
plot(retinaX,'-r','LineWidth',2)
hold on;
plot(retinaY,'-b','LineWidth',2)
title('Retinas views- ball x-displacement for actuators');
xlabel('Iterations');
ylabel('x-Coords');
legend('Actuator1','Actuator2','Location','Best')

%ball pos should eventually centre/converge around x=length/2
figure;
plot(ballX,'-b','LineWidth',2)
title('Rick Code Ball Catching');


