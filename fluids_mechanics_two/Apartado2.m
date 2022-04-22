clc 
close all
%Apartado 2%
n1 = 179*3.141592/180;
n2 = 120*3.141592/180;
n3 = 90*3.141592/180;
n4 = 30*3.141592/180;
n5 = 5*3.141592/180;
delt=0.01;

%Variaciones de angulo para RK2%

figure (1)
[T,y] = RK2(delt,0,8,[n1 0],1/2,1/2,@ang1);
plot(T,y(1,:),'b')

[T,y] = RK2(delt,0,8,[n2 0],1/2,1/2,@ang1);
hold on
plot(T,y(1,:),'m')

[T,y] = RK2(delt,0,8,[n3 0],1/2,1/2,@ang1);
hold on
plot(T,y(1,:),'g')

[T,y] = RK2(delt,0,8,[n4 0],1/2,1/2,@ang1);
hold on
plot(T,y(1,:),'r')

[T,y] = RK2(delt,0,8,[n5 0],1/2,1/2,@ang1);
hold on

plot(T,y(1,:),'k')
xlabel('Tiempo (s)')
ylabel('Angulo (Rad)')
legend('179º','120º','90º','30º','5º')

%Variaciones de angulo para RK4%

figure (2)
[T,y] = RK4(delt,0,8,[n1 0],@ang1);
plot(T,y(1,:),'b')

[T,y] = RK4(delt,0,8,[n2 0],@ang1);
hold on
plot(T,y(1,:),'m')

[T,y] = RK4(delt,0,8,[n3 0],@ang1);
hold on
plot(T,y(1,:),'g')

[T,y] = RK4(delt,0,8,[n4 0],@ang1);
hold on
plot(T,y(1,:),'r')

[T,y] = RK4(delt,0,8,[n5 0],@ang1);
hold on

plot(T,y(1,:),'k')
xlabel('Tiempo (s)')
ylabel('Angulo (Rad)')
legend('179º','120º','90º','30º','5º')

%Variaciones de angulo para Heun%

figure (3)
[T,y] = Heun(delt,0,8,[n1 0],@ang1);
plot(T,y(1,:),'b')

[T,y] = Heun(delt,0,8,[n2 0],@ang1);
hold on
plot(T,y(1,:),'m')

[T,y] = Heun(delt,0,8,[n3 0],@ang1);
hold on
plot(T,y(1,:),'g')

[T,y] = Heun(delt,0,8,[n4 0],@ang1);
hold on
plot(T,y(1,:),'r')

[T,y] = Heun(delt,0,8,[n5 0],@ang1);
hold on

plot(T,y(1,:),'k')
xlabel('Tiempo (s)')
ylabel('Angulo (Rad)')
legend('179º','120º','90º','30º','5º')
