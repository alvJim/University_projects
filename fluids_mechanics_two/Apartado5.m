clc
close all


[T,y] = RK4(0.01,0,10,[179*pi/180 0],@ang1); %Sin amortiguacion

[T,y] = RK4c(0.01,0,10,[179*pi/180 0],0.01,@ang4); %Con amortiguacion 
[m,n] = size(y);

Tm=9.81*cos(y(1,:))+0.5*(y(2,:)).^2;

MaxTm= max(Tm);



Emax=55*10.^6;
A=1/(10.^4);
m=Emax*A/MaxTm;

disp(MaxTm);
disp(m);

