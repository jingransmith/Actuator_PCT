function F = paramfunJS2(x,len,y1,y2)
%x(1)=angle1, x(2)=angle3

F(1) = y1 + len*sin(x(1)-(pi/2)) - y2*sin(x(2)); %0 on RHS %y
F(2) = len*cos(x(1)-(pi/2)) + y2*cos(x(2)) - len; %x
end