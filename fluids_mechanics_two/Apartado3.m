clc
close all
N = 179;%donde los n serán los valores del angulo theta

for i = 0:0.1:N
 N1 =N-i;
 N2 = N1 * pi/180;
  
  [T,y] = RK4(0.01,0,8,[N2 0],@ang1);
  [T1,y1] = metodoan(0.01,0,8,N2,@expr);
  
  diferencia = y(1,:) - y1(1,:);
  modulo1 = sqrt(sum(diferencia.^2));
  modulo2 = sqrt(sum(y1(1,:).^2));
  
  error = (modulo1/modulo2)*100;
  
   if error<16 && error>14
   
   Tta = N1;
  
   end
 
end
disp(Tta);


 figure(1);
  [T1,y1] = metodoan(0.01,0,8,(15*pi/180),@expr);
  
   plot(T1,y1(1,:),'r')

     
  [T,y] = RK4(0.01,0,8,[(179*pi/180) 0],@ang1);
  
    hold on
    plot(T,y(1,:),'g')
  
   [T,y] = RK4(0.01,0,8,[(15*pi/180) 0],@ang1);
   
     hold on
     plot(T,y(1,:),'k')
     
xlabel('Tiempo (s)')
ylabel('Angulo (Rad)')
legend('Sol.An.;15º','RK4;179º','RK4;15º')
