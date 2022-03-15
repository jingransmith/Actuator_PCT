function F = paramfunJS_edited(x,len,y1,y2)
%x(1)=angle1, x(2)=angle3

%len=x(3), y2=-1, y1=1

F(1) = len*cos(x(1)) + y2*sin(x(2)) - y1; %0 on RHS
F(2) = y2*cos(x(2)) + len*sin(x(1)) - len;
end