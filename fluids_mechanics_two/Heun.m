function [T,y] = Heun(deltat,to,tf,yo,derecho)

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
k2 = derecho(T(i) + (deltat/3), y(:,i) + (deltat/3)*k1);
k3 = derecho(T(i) + (2*deltat/3), y(:,i) + (2*deltat/3)*k2);    
y(:,i+1) = y(:,i) + (deltat/4)*(k1 + 3*k3);
    
end