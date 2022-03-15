%% Rick's Code for TG 17-07-2021 

clc; close all; clear all

%% Variables
i = 0; l = 0; dt = 0.05;

% Not Used Here
% halfsecsamp = floor(0.5/dt);

% I generated Random Number Generator between 0 and 1
rand_x = rand_num();
x = rand_x;

cv = x.*8+18.0; %sensor inputs?
angley = (pi/2.0)./(x*0.8+1.2);
anglez = ((x*200)./100.0)*(pi/8.0)-(pi/8.0);
hc = cos(angley).*cv;
vc = sin(angley).*cv;
fx = ((x.*100-50.0)/50.0)*6.0+45.0;
bhp =0; bvp = 0;

fz = 0; fxp = fx; fzp = 0; fy = 0; 
ref1= 0.0185; ref2 = 0; up = 0; vp = 0; ldp = 0; 
fvel = 0; facc = 0; lvel = 0; lacc = 0; 
fgain = 2000.0; lgain=2000.0; fslow=500; lslow = 1000.0;

% Gravity
g=10.0; 

% 3 x 4 Matrix of zeros
delay = zeros(3,4);
% % delay = [0,0,0,0,0;
% %          0,0,0,0,0;
% %          0,0,0,0,0];
drag = 0;
d = 1; % For Loop to iterate indefinitely
xx = 0;
while (d > 0) 
    xx = xx + 1;        
    % Iterating over bh, bv
    %displacements
    bh = bhp + hc*dt; % dt,hc are constants
    bhp = bh;
    bv = bvp + vc*dt; % vc,dt are constants
    bvp = bv;
    
    % Iterating over velocities going backwards
    vc = vc - (g*dt);     %sin of Velocity
    hc = hc - (drag*dt);  %cos of Velocity
    
    bx = cos(anglez)*bh;
    bz = sin(anglez)*bh;
    by = bv;
    
    
    % u = Lateral ball angle
    fa = fz/fx;
    ba = (fz-bz)/(fx-bx);
    u = fa-ba;
    ld = (fz-bz)/(fx-bx);
    
    
    % v = vertical angle
    % Distance between fielder and ball in 2D
    xprime = sqrt((fx-bx)*(fx-bx)+(fz-bz)*(fz-bz));
    yprime = by;
    v = yprime/xprime;
    
    p1 = v-vp;
    p2 = ld-ldp;
    vp = v;
    ldp = ld;
    
    % Matlab Cannot Iterate over 0 index, hence 1-4 with Minus 1
    for j = 1:1:4
        delay(2,j+1) = delay(2,j);
        delay(3,j+1) = delay(3,j);
    end
    
    noisy = noise();
    delay(2,1) = p1 + noisy(1)/30000.0;
    delay(3,1) = p2 + noisy(1)/30000.0;
    
    %console.log(p1);
    %console.log(noise[l]);
    
    % Increment l till 400 with 5 and reset after 400
    l = l + 5;
    if (l > 399)
        l = 0;
    end
%     

    fvel = fvel + (fgain * (ref1-delay(2,1))-fvel)/fslow; %fielder x velocity
    lvel = lvel + (lgain * (ref2-delay(3,1))-lvel)/lslow; %fielder z verlocity
     %fvel = fvel + (fgain * (ref1-delay(1,1))-fvel)/fslow; %fielder x velocity
    %lvel = lvel + (lgain * (ref2-delay(1,1))-lvel)/lslow; %fielder z verlocity
    %how did get velocity from basically only references? 
    
%     Thresholding
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
%     
    fx = fx - fvel;
    fz = fz + lvel;
    
%     
%     runrate = sqrt((fx-fxp)*(fx-fxp)+(fz-fzp)*(fz-fzp));
    
    fxp = fx; 
    fzp = fz;
    
    %console.log(fx-bx);
    
    % Stop the Code Execution
    if abs(fx-bx) < 1.5
        d = 0;
    end
    if (by < 0)
        d = 0;
    end
    
    % Save Data for Maximum of 199 Points
    if (i < 200 )
        i = i + 1;
        retinaX(i) = u;
        retinaY(i) = v;
        fielderX(i) = fx;
        fielderZ(i) = fz;
        ballX(i) = bx;
        ballY(i) = by;
        ballZ(i) = bz;
        Fvel(i)=fvel;
        Lvel(i)=lvel;
%         runningRate(i) = runrate/dt;
        NumOfDataPoints = i;
    end
    
% %Plotting the code
% plot(xx,bh,'-or');
% hold on;
% plot(xx,hc,'-ob');
% plot(xx,vc,'-ok');
% plot(xx,hc,'-og');
end

figure;
plot(fielderX,'-r','LineWidth',2)
hold on;
plot(ballX,'-b','LineWidth',2)
title('Rick Code Ball Catching');
xlabel('Iterations');
ylabel('X-Coords');
legend('Fielder','Ball','Location','Best')

figure;
plot(fielderZ,'-r','LineWidth',2)
hold on;
plot(ballZ,'-b','LineWidth',2)
title('Rick Code Ball Catching');
xlabel('Iterations');
ylabel('Z-Coords');
legend('Fielder','Ball','Location','Best')


figure;
 plot3(fielderX,fielderZ,zeros(1,size(fielderX,2)),'-r','LineWidth',2)
 hold on;
plot3(ballX,ballZ,ballY,'-b','LineWidth',2)
title('Fielder Ball Catching with Perceptual Control');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
 legend('Fielder','Ball','Location','Best')

