clc
close all

pico=0;
direc=0;
[m,n] = size(y);
amort=0;

for c=0:0.01:1.5

    [T,y] = RK4c(0.01,0,15,[179*pi/180 0],c,@ang4);

     for i=1:1501

      if i==1
       j=1;
      else
      if i==2
           j=1;
       else
            j=i-1;
        
      end
      end
 
    if y(1,i)>y(1,j) && direc==0
   
        pico=pico+1;
        direc=1;
        
    else 
        if y(1,i) < y (1,j) && direc == 1
          pico = pico +1;
          direc=0;
        end
        
        end
    
    if pico==6
      ang=y(1,i);
      if ang > 2.89 && ang < 2.91
        amort=c;
      end
    end
  
      end

    end
        




[T,y] = RK4c(0.001,0,12,[179*pi/180 0],amort,@ang4);

plot(T,y(1,:),'b')

[T,y] = RK4(0.001,0,12,[179*pi/180 0],@ang1);

hold on
plot(T,y(1,:),'r')

xlabel('Tiempo(s)') 
ylabel('Deflexion angular (rad)')
legend ('Amortiguado','No amortiguado')



