%Cálculo de trayectorias

h = 2000 ;
alfa = 6*pi/180 ;
gamma = 0*pi/180 ;
V = 102.6536 ;
pospalanca = 0.1718 ;
delta = -0.0421 ;
u_ini = V*cos(alfa) ;
w_ini = V*sin(alfa) ;
q_ini = 0 ;
theta_ini = gamma + alfa ;
x_ini = 0 ;
z_ini = -h ;

[T, X, Resultados] = sim('Trabajo', [0 10000]);

