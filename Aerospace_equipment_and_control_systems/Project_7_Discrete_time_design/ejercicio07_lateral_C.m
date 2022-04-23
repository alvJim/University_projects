% -----------------------------------------------------------------------
% Ficheros: E8_ControllatgitudinalMIMO.m
% E8_ControllatgitudinalMIMOsim.mdl
%
% Fecha: 2 de noviembre de 2021
% Asignatura: ESCA
% -----------------------------------------------------------------------
clear
dt =0.001;
%1. La dinámica de una aeronave se ha linealizado para una condición de vuelo
% recto y nivelado (1000 m y 153 m/s). Las matrices que corresponden al modelo
% linealizado de la dinámica lateral se indican a continuación.
% x = [ vT alfa theta q ]'
A_lat = [-0.0191 2.5172 -9.8036 -0.2540
 -0.0008 -0.9713 0.0000 0.9126
 0.0000 0.0000 0.0000 1.0000
 0.0000 -0.7234 0.0000 -1.2607];
% u = [delta_TH delta_e]'
B_lat = [ 0.1128 0.0298
 0.0000 -0.0022
 -0.0000 0.0000
 0.0000 -0.1743];
%C_lat = [1 0 0 0 % vT
% 0 0 1 0]; % theta
C_lat = [1 0 0 0 % vT
 0 0 180/pi 0]; % theta
D_lat = zeros(2);
SN = {'vT','alfa','theta','q'};
IN = {'delta_TH','delta_e'};
ON = {'vT','theta'};
sys_lat = ss(A_lat,B_lat,C_lat,D_lat,'StateName',SN,'InputName',IN,'OutputName',ON)
% Actuadores para simulación
% Elevador
tau_r = 1/20;
delta_e_lim = [-25 25]; % límites de posición (rad)
delta_e_dot_lim = [-90 90]; % límites de velocidad (rad/s)
% Throttle
tau_a = 0.5;
delta_th_lim = [-0.2 0.8]*100; % límites (%)
delta_th_dot_lim = [-0.24 0.24]*100; % límites de velocidad (%/s)
%--------------------------------------------------------------------------
% Realizar un diseño de un controlador MIMO 2x2 por asignación de polos mediante
% realimentación del vector de estado para control de dinámica latgitudinal.
% En cada caso habrá que elegir las variables controladas.
% Utilizar para ello las técnicas siguientes:
% Sistema en tiempo discreto
% tau_min = 1/1.37 modo corto periodo
 % aprox. Tm = tau_min/100
Tm = 0.003;
I = eye(4);
F_lat = A_lat*Tm + I;
G_lat = B_lat*Tm;
sysd_lat = ss(F_lat,G_lat,C_lat,D_lat,Tm);
% o mediante la función c2d:
sysd_lat = c2d(sys_lat,Tm,'zoh') % 'tustin': transformacion bilineal;
step(sys_lat,sysd_lat); grid
% Controlabilidad y Observabilidad:
Mc = [F_lat F_lat*G_lat F_lat^2*G_lat F_lat^3*G_lat];
% o con la función de Matlab:
Mc = ctrb(F_lat,G_lat);
if rank(Mc)==length(F_lat)
 disp('** Sistema controlable');
else
 disp('** Sistema no contrable');
end
Mo = [C_lat; C_lat*F_lat; C_lat*F_lat^2; C_lat*F_lat^3];
% o con la función d Matlab:
Mo = obsv(F_lat,C_lat);
if rank(Mo)==length(F_lat)
 disp('** Sistema observable');
else
 disp('** Sistema no observable');
end
%{
 Estabilidad del sistema en lazo cerrado en el plano Z:
 1) Para que el sistema sea estable los polos deben estar dentro del círculo
 unidad. Cualquier polo externo al círculo unidad hace al sistema inestable.
 2) Si un polo simple se presenta en z=1 o un par de polos complejos
 conjugados se presentan en el círculo unidad el sistema es críticamente
 estable.
 3) Los ceros en lazo cerrado no afectan a la estabilidad absoluta y
 pueden locarlizarse en cualquier parte.
%}
while 1
 opt = menu('ESCA: Ej.7 Diseño discreto control latgitudinal', ...
 '1. Control modal - Observador de Luenberger',...
 '2. Diseño LQG (LQR + Filtro de Kalman)',...
 '3. Acción integral: Control Modal + Filtro de Kalman', ...
 '4. Acción integral: LQG (LQR + Filtro de Kalman)',...
 '5. Salir');
 switch opt
 case 1 % a) Control modal - Observador de Luenberger
 % Recordando que la relación entre el plano s y el plano z es
 % z = exp(Tm*s) se pueden reutilizar los diseños previos en tiempo
 % continuo:
 polos_c = exp(Tm*[-1 -3 -4 -8]');
 Kc = place(F_lat,G_lat,polos_c);
 Fyr = F_lat - G_lat*Kc;
 Cyr = C_lat - D_lat*Kc;
 PreC=[Cyr*(I-Fyr)^(-1)*G_lat+D_lat]^(-1);
 Gyr = G_lat*PreC;
 Dyr = D_lat*PreC;
 sysd_lat_lc = ss(Fyr,Gyr,Cyr,Dyr,Tm)
 pole(sysd_lat_lc)
 tzero(sysd_lat_lc)
 dcgain(sysd_lat_lc)
 zpk(sysd_lat_lc)
 damp(sysd_lat_lc)
 step(sysd_lat_lc); grid

 polos_o = exp(Tm*2*[-1 -3 -4 -8]');
 Ko_d = place(F_lat',C_lat',polos_o);
 Ko_d = Ko_d';
 case 2 % Diseño LQG (LQR + Filtro de Kalman)
 V1 = [0.1 0; 0 0.1];
 V2 = [2 0; 0 2];
 Ko_d = dlqe(F_lat,G_lat,C_lat,V1,V2);
 q1 = 5e7; % vT
 q2 = 5e7; % alfa
 q3 = 5e7; % theta
 q4 = 5e7; % q
 r1 = 1e4; % dTH
 r2 = 1e4; % de
 Qc = diag([q1 q2 q3 q4]);
 Rc = diag([r1 r2]);
 Kc = dlqr(F_lat,G_lat,Qc,Rc);
 Fyr = F_lat - G_lat*Kc;
 Cyr = C_lat - D_lat*Kc;
 PreC = [Cyr*(I-Fyr)^(-1)*G_lat+D_lat]^(-1);
 case 3 % Acción integral: Control Modal + Filtro de Kalman
 F_lat_a = [F_lat zeros(4,2);-Tm*C_lat eye(2)];
 G_lat_a = [G_lat; -Tm*D_lat];
 polos_a = exp(Tm*[-1 -1.5 -2 -2.5 -3 -4]);
 V1 = [0.1 0; 0 0.1];
 V2 = [2 0; 0 2];
 Ko_d = dlqe(F_lat,G_lat,C_lat,V1,V2);
 Kc_a = place(F_lat_a,G_lat_a,polos_a);
 Kc = Kc_a(:,1:4);
 Ki = -Kc_a(:,5:6);
 case 4 % Acción integral: LQG (LQR + Filtro de Kalman)
 F_lat_a = [F_lat zeros(4,2);-Tm*C_lat eye(2)];
 G_lat_a = [G_lat; -Tm*D_lat];
 q1 = 5e5;
 q2 = 5e5;
 q3 = 5e5;
 q4 = 5e5;
 q5 = 5e5;
 q6 = 5e5;
 r1 = 1e4;
 r2 = 1e4;
 Qc = diag([q1 q2 q3 q4 q5 q6]);
 Rc = diag([r1 r2]);
 Kc_a = dlqr(F_lat_a,G_lat_a,Qc,Rc);
 Kc = Kc_a(:,1:4);
 Ki = -Kc_a(:,5:6);
 V1 = [0.1 0; 0 0.1];
 V2 = [2 0; 0 2];
 Ko_d = dlqe(F_lat,G_lat,C_lat,V1,V2);
 case 5 % Salir
 break;
 end
end