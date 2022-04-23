%Bloque atmósfera
g = 9.80665 ;
R = 287.0531 ;
L = 0.0065 ;
h_trop = 11000 ;
h_strat = 20000 ;
rho0 = 1.225 ;
Temp_0 = 288.15 ;
Temp_trop = 216.65 ;
rho_trop = 0.36391808 ;
rho_strat = 0.08803489 ;

%Parámetros aeronave
pos_cdg = [-7.5, 0, 0] ;
pos_ala = [-6.9, 0, 0.4] ;
pos_cola = [-18.35, 0, -4.6] ;
pos_motor = [-13.4, 0, -1.1] ;
masa = 19500 ;
Iy = 191650 ;
Tmax = 82000 ;
i_motor = 3*pi/180 ;

S = 45.4;
Sh = 10.1;
c = 2.3;
CLalfaw = 4.206;
CL0 = 0.2987;
CLalfah = 3.552;
CLdelta = 1.916;
CD0 = 0.016;
K = 0.0556;
CMalfa = 1.321;
CM0 = -0.0813;


%Parámetros para los que queremos calcular el punto de equilibrio deseado
alfa = 6*pi/180 ;
h = 2000 ;
gamma = 0*pi/180;
q_ini = 0 ;
theta_ini = gamma + alfa ;
x_ini = 0 ;
z_ini = -h ;

delta = 0*pi/180 ;
pospalanca = 0.5 ;

BuscaEquilibrio
 