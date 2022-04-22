function [T,y] = RK4(deltat,to,tf,yo,derecho)

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
k2 = derecho(T(i) + (deltat/2), y(:,i) + (deltat/2)*k1); 
k3 = derecho(T(i) + (deltat/2), y(:,i) + (deltat/2)*k2);
k4 = derecho(T(i) + deltat, y(:,i) + deltat*k3);  
y(:,i+1) = y(:,i) + (deltat/6)*(k1 + 2*k2 + 2*k3 + k4);
    
end