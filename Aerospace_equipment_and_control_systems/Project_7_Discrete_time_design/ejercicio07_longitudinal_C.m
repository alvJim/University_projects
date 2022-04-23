% -----------------------------------------------------------------------
% Ficheros: E8_ControlLongitudinalMIMO.m
% E8_ControlLongitudinalMIMOsim.mdl
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
A_lon = [-0.0191 2.5172 -9.8036 -0.2540
 -0.0008 -0.9713 0.0000 0.9126
 0.0000 0.0000 0.0000 1.0000
 0.0000 -0.7234 0.0000 -1.2607];
% u = [delta_TH delta_e]'
B_lon = [ 0.1128 0.0298
 0.0000 -0.0022
 -0.0000 0.0000
 0.0000 -0.1743];
%C_lon = [1 0 0 0 % vT
% 0 0 1 0]; % theta
C_lon = [1 0 0 0 % vT
 0 0 180/pi 0]; % theta
D_lon = zeros(2);
SN = {'vT','alfa','theta','q'};
IN = {'delta_TH','delta_e'};
ON = {'vT','theta'};
sys_lon = ss(A_lon,B_lon,C_lon,D_lon,'StateName',SN,'InputName',IN,'OutputName',ON)
% Actuadores para simulación
% Elevador
tau_e = 1/20;
delta_e_lim = [-25 25]; % límites de posición (rad)
delta_e_dot_lim = [-90 90]; % límites de velocidad (rad/s)
% Throttle
tau_th = 0.5;
delta_th_lim = [-0.2 0.8]*100; % límites (%)
delta_th_dot_lim = [-0.24 0.24]*100; % límites de velocidad (%/s)
%--------------------------------------------------------------------------
% Realizar un diseño de un controlador MIMO 2x2 por asignación de polos mediante
% realimentación del vector de estado para control de dinámica longitudinal.
% En cada caso habrá que elegir las variables controladas.
% Utilizar para ello las técnicas siguientes:
% Sistema en tiempo discreto
% tau_min = 1/1.37 modo corto periodo
 % aprox. Tm = tau_min/100
Tm = 0.003;
I = eye(4);
F_lon = A_lon*Tm + I;
G_lon = B_lon*Tm;
sysd_lon = ss(F_lon,G_lon,C_lon,D_lon,Tm);
% o mediante la función c2d:
sysd_lon = c2d(sys_lon,Tm,'zoh') % 'tustin': transformacion bilineal;
step(sys_lon,sysd_lon); grid
% Controlabilidad y Observabilidad:
Mc = [F_lon F_lon*G_lon F_lon^2*G_lon F_lon^3*G_lon];
% o con la función de Matlab:
Mc = ctrb(F_lon,G_lon);
if rank(Mc)==length(F_lon)
 disp('** Sistema controlable');
else
 disp('** Sistema no contrable');
end
Mo = [C_lon; C_lon*F_lon; C_lon*F_lon^2; C_lon*F_lon^3];
% o con la función d Matlab:
Mo = obsv(F_lon,C_lon);
if rank(Mo)==length(F_lon)
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
 opt = menu('ESCA: Ej.7 Diseño discreto control longitudinal', ...
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
 Kc = place(F_lon,G_lon,polos_c);
 Fyr = F_lon - G_lon*Kc;
 Cyr = C_lon - D_lon*Kc;
 PreC=[Cyr*(I-Fyr)^(-1)*G_lon+D_lon]^(-1);
 Gyr = G_lon*PreC;
 Dyr = D_lon*PreC;
 sysd_lon_lc = ss(Fyr,Gyr,Cyr,Dyr,Tm)
 pole(sysd_lon_lc)
 tzero(sysd_lon_lc)
 dcgain(sysd_lon_lc)
 zpk(sysd_lon_lc)
 damp(sysd_lon_lc)
 step(sysd_lon_lc); grid

 polos_o = exp(Tm*2*[-1 -3 -4 -8]');
 Ko_d = place(F_lon',C_lon',polos_o);
 Ko_d = Ko_d';
 case 2 % Diseño LQG (LQR + Filtro de Kalman)
 V1 = [0.1 0; 0 0.1];
 V2 = [2 0; 0 2];
 Ko_d = dlqe(F_lon,G_lon,C_lon,V1,V2);
 q1 = 5e7; % vT
 q2 = 5e7; % alfa
 q3 = 5e7; % theta
 q4 = 5e7; % q
 r1 = 1e4; % dTH
 r2 = 1e4; % de
 Qc = diag([q1 q2 q3 q4]);
 Rc = diag([r1 r2]);
 Kc = dlqr(F_lon,G_lon,Qc,Rc);
 Fyr = F_lon - G_lon*Kc;
 Cyr = C_lon - D_lon*Kc;
 PreC = [Cyr*(I-Fyr)^(-1)*G_lon+D_lon]^(-1);
 case 3 % Acción integral: Control Modal + Filtro de Kalman
 F_lon_a = [F_lon zeros(4,2);-Tm*C_lon eye(2)];
 G_lon_a = [G_lon; -Tm*D_lon];
 polos_a = exp(Tm*[-1 -1.5 -2 -2.5 -3 -4]);
 V1 = [0.1 0; 0 0.1];
 V2 = [2 0; 0 2];
 Ko_d = dlqe(F_lon,G_lon,C_lon,V1,V2);
 Kc_a = place(F_lon_a,G_lon_a,polos_a);
 Kc = Kc_a(:,1:4);
 Ki = -Kc_a(:,5:6);
 case 4 % Acción integral: LQG (LQR + Filtro de Kalman)
 F_lon_a = [F_lon zeros(4,2);-Tm*C_lon eye(2)];
 G_lon_a = [G_lon; -Tm*D_lon];
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
 Kc_a = dlqr(F_lon_a,G_lon_a,Qc,Rc);
 Ko_d = dlqe(F_lon,G_lon,C_lon,V1,V2);
 Kc = Kc_a(:,1:4);
 Ki = -Kc_a(:,5:6);
 V1 = [0.1 0; 0 0.1];
 V2 = [2 0; 0 2];

 case 5 % Salir
 break;
 end
end