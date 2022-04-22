function [T,y] = RK2(deltat,to,tf,yo,a1,a2,derecho)

npasos = floor((tf - to)/deltat);

T = zeros(npasos + 1,1);
y = zeros(2,npasos + 1);

T(1) = to;
y(1,1) = yo(1);
y(2,1) = yo(2);

for i = 1:npasos
    
T(i+1) = T(i) + deltat;
    
end

for i = 1:npasos

k1 = derecho(T(i),y(:,i));
k2 = derecho(T(i) + deltat, y(:,i) + deltat*k1);    
y(:,i+1) = y(:,i) + deltat*(a1*k1 + a2*k2);
    
end