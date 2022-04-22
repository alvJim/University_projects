function [T1,y1] = metodoan(deltat,to,tf,zo,funcion)
  
  npasos = floor((tf - to)/deltat);

T1 = zeros(npasos + 1,1);
y1 = zeros(1,npasos + 1);

T1(1) = to;
y1(1,1) = zo;

for i = 1:npasos
    
T1(i+1) = T1(i) + deltat;
    
end
  
  for i = 1:npasos
    
    y1(1,i+1) = funcion(T1(i),zo);
  end
end