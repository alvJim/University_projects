clc
close all

%Apartado 1%

%Delta = 0,1%
n=179*3.141592/180;

figure (1)
[T,y] = RK2(0.1,0,8,[n 0],1/2,1/2,@ang1);

plot(T,y(1,:))

[T,y] = RK4(0.1,0,8,[n 0],@ang1);

hold on
plot(T,y(1,:),'r')

[T,y] = Heun(0.1,0,8,[n 0],@ang1);

hold on
plot(T,y(1,:),'k')

xlabel('Tiempo (s)')
ylabel('Angulo (Rad)')
legend('RK2','RK4','Heun')


%Delta = 0,01%

figure (2)
[T,y] = RK2(0.01,0,8,[n 0],1/2,1/2,@ang1);

plot(T,y(1,:))

[T,y] = RK4(0.01,0,8,[n 0],@ang1);

hold on
plot(T,y(1,:),'r')

[T,y] = Heun(0.01,0,8,[n 0],@ang1);

hold on
plot(T,y(1,:),'k')

xlabel('Tiempo (s)')
ylabel('Angulo (Rad)')
legend('RK2','RK4','Heun')


%Delta = 0,001%

figure (3)
[T,y] = RK2(0.001,0,8,[n 0],1/2,1/2,@ang1);

plot(T,y(1,:))

[T,y] = RK4(0.001,0,8,[n 0],@ang1);

hold on
plot(T,y(1,:),'r')

[T,y] = Heun(0.001,0,8,[n 0],@ang1);

hold on
plot(T,y(1,:),'k')

xlabel('Tiempo (s)')
ylabel('Angulo (Rad)')
legend('RK2','RK4','Heun')

%Delta = 0,0001%

figure (4)
[T,y] = RK2(0.0001,0,8,[n 0],1/2,1/2,@ang1);

plot(T,y(1,:))

[T,y] = RK4(0.0001,0,8,[n 0],@ang1);

hold on
plot(T,y(1,:),'r')

[T,y] = Heun(0.0001,0,8,[n 0],@ang1);

hold on
plot(T,y(1,:),'k')

xlabel('Tiempo (s)')
ylabel('Angulo (Rad)')
legend('RK2','RK4','Heun')
