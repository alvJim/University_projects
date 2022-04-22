
clc 
close all
%pi=3.14159265359;

j=0;

for i=0:0.01:pi
  
j=j+1;
suma = zeros (1,85);
  for k=0:85
    num=factorial(2*k);
    den=factorial(k);
    elev=2*k;
    
    suma(1,k+1)=(num/(2.^(elev)*den.^2)).^(2)*sin(i/2).^(elev);
    
    
  end
  
  S=sum(suma);
  f(1,j)=S;
  r(1,j)=i*180/pi;
  
end

plot(r(1,:),f(1,:))
grid minor 
xlabel('Angulo (Grados)') 
ylabel('T/To(s/s)')
