% -----------------------------------------------------------------------
% Ficheros: E5_ControlLateralMIMO.m
% E5_ControlLateralMIMOsim.mdl
%
% Fecha: 1 de octubre de 2021
% Asignatura: ESCA
% -----------------------------------------------------------------------
clear
dt =0.001;
%1. La dinámica de una aeronave se ha linealizado para una condición de vuelo
% recto y nivelado (1000 m y 153 m/s). Las matrices que corresponden al modelo
% linealizado de la dinámica lateral se indican a continuación.
% x = [ beta p r phi ]'
A_lat = [ -0.2723 0.0640 0.0469 -0.9918
 0.0000 0.0000 1.0000 0.0470
 -28.2010 0.0000 -3.2574 0.6348
 7.2278 0.0000 -0.0335 -0.4542];
% u = [delta_a delta_r]'
B_lat = [ 0.0002 0.0007
 0.0000 0.0000
 -0.6222 0.1152
 -0.0268 -0.0584];
C_lat = [180/pi 0 0 0 % beta
 0 0 0 180/pi]; % phi
D_lat = zeros(2);
SN = {'beta','p','r','phi'};
IN = {'delta_a','delta_r'};
ON = {'beta_m','phi_m'};
sys_lat = ss(A_lat,B_lat,C_lat,D_lat,'StateName',SN,'InputName',IN,'OutputName',ON)
% Actuadores para simulación
% Alerones
tau_a = 1/20;
delta_a_lim = [-30 30]; % límites de posición (grados)
delta_a_dot_lim = [-80 80]; % límites de velocidad (grados/s)
% Timon (misma dinamica que los alerones)
tau_r = tau_a;
delta_r_lim = delta_a_lim; % límites de posición (rad)
delta_r_dot_lim = delta_a_dot_lim; % límites de velocidad (rad/s)
%--------------------------------------------------------------------------
% Realizar un diseño de un controlador MIMO 2x2 por asignación de polos mediante
% realimentación del vector de estado para control de dinámica lateral.
% En cada caso habrá que elegir las variables controladas.
% Utilizar para ello las técnicas siguientes:
% Controlabilidad y Observabilidad:
Mc = [B_lat A_lat*B_lat A_lat^2*B_lat A_lat^3*B_lat];
% o con la función de Matlab:
Mc = ctrb(A_lat,B_lat);
if rank(Mc)==length(A_lat)
 disp('** Sistema controlable');
else
    disp('** Sistema no contrable');
end
Mo = [C_lat; C_lat*A_lat; C_lat*A_lat^2; C_lat*A_lat^3];
% o con la función d Matlab:
Mo = obsv(A_lat,C_lat);
if rank(Mo)==length(A_lat)
 disp('** Sistema observable');
else
 disp('** Sistema no observable');
end
%{
CRITERIOS DE DISEÑO
1. Al diseñar un controlador, obsérvese que si los polos dominantes del controlador
 se situan alejados a la izquierda del eje imaginario, los elementos de la matriz
 de ganancia de realimentación del estado Kc se harán grandes. Grandes valores de
 la ganancia Kc harán que la salida del actuador sea también grande, de manera que
 puede dar lugar a saturación de los actuadores.
2. También, al situar los polos del observador suficientemente alejados a la izquierda
 del eje imaginario, el conjunto controlador-observador puede hacerse inestable,
 aunque el sistema en lazo cerrado sea estable.
3. Si el controlador-observador se hace inestable, se mueven los polos del observador
 a la derecha en el semiplano izquierdo del plano s hasta que se estabilice.
También,
 la localización de los polos en lazo cerrado deseados puede tener que modificarse.
4. Obsérvese que si los polos del observador se colocan lejos a la izquierda del eje
 imaginario, el ancho de banda del observador aumentará y originará problemas de
 ruido. El requisito general es que el ancho de banda debería ser suficientemente
 bajo para que el ruido de los sensores que miden y(t) y se realimenta al observador
 no sea un inconveniente.
En resumen, los polos del observador deben ser bastante más rápidos que los de la
planta en lazo cerrado, pero no tanto como para que generen problemas de inestabilidad
y ruido.
Como regla general, los polos del observador deben ser de dos a cinco veces más
rápidos
que los polos del controlador para asegurarse que el error de observación (error de
estimación) converge a cero rápidamente.
%}
while 1
 opt = menu('ASAN: Ejercicio 5 Control lateral MIMO', ...
 '1. Control Modal - Observador de Luenberger',...
 '2. Diseño LQG (LQR + Filtro de Kalman)', ...
 '3. Acción integral: Control Modal + Filtro de Kalman',...
 '4. Acción integral: LQG (LQR + Filtro de Kalman)',...
 '5. Versión en tiempo discreto',...
 '6. Salir');

 switch opt
 case 1 % a) Control modal - Observador de Luenberger
 polos_c = [-1 -2 -3 -4]';
 Kc = place(A_lat,B_lat,polos_c);
 Ayr = A_lat - B_lat*Kc;
 Cyr = C_lat - D_lat*Kc;
 PreC=[Cyr*(-Ayr)^(-1)*B_lat+D_lat]^(-1);
 Byr = B_lat*PreC;
 Dyr = D_lat*PreC;
 sys_lat_lc = ss(Ayr,Byr,Cyr,Dyr)
 pole(sys_lat_lc)
 zero(sys_lat_lc)
 dcgain(sys_lat_lc)
 zpk(sys_lat_lc)
 damp(sys_lat_lc)
 step(sys_lat_lc); grid

 polos_o = polos_c*2;
 Ko = place(A_lat',C_lat',polos_o);
 Ko = Ko';
 case 2 % Diseño LQG (LQR + Filtro de Kalman)
 V1 = [0.1 0; 0 0.1];
 V2 = [2 0; 0 2];
 Ko = lqe(A_lat,B_lat,C_lat,V1,V2);
 q1 = 5e5; 
  q2 = 5e5;
  q3 = 5e5;
  q4 = 5e5;
  r1 = 1e1;
  r2 = 1e1;
 Qc = diag([q1 q2 q3 q4]);
 Rc = diag([r1 r2]);
 Kc = lqr(A_lat,B_lat,Qc,Rc);
 Ayr = A_lat - B_lat*Kc;
 Cyr = C_lat - D_lat*Kc;
 PreC = [Cyr*(-Ayr)^(-1)*B_lat+D_lat]^(-1);
 case 3 % Acción integral: Control Modal + Filtro de Kalman
 A_lat_a = [A_lat zeros(4,2); -C_lat zeros(2,2)];
 B_lat_a = [B_lat; -D_lat];
 %polos_a = [-1 -2 -3 -5 -5 -10];
 polos_a = [-3 -4 -5 -6 -7 -12];
 Kc_a = place(A_lat_a,B_lat_a,polos_a);
 Kc = Kc_a(:,1:4);
 Ki = -Kc_a(:,5:6);
 V1 = [0.1 0; 0 0.1];
 V2 = [2 0; 0 2];
 Ko = lqe(A_lat,B_lat,C_lat,V1,V2);
 case 4 % Acción integral: LQG (LQR + Filtro de Kalman)
 A_lat_a = [A_lat zeros(4,2); -C_lat zeros(2,2)];
 B_lat_a = [B_lat; -D_lat];
 q1 = 5e5;
 q2 = 5e5;
 q3 = 5e5;
 q4 = 5e5;
 q5 = 5e5;
 q6 = 5e5;
 r1 = 1e1;
 r2 = 1e1;
 Qc = diag([q1 q2 q3 q4 q5 q6]);
 Rc = diag([r1 r2]);
 Kc_a = lqr(A_lat_a,B_lat_a,Qc,Rc);
 Kc = Kc_a(:,1:4);
 Ki = -Kc_a(:,5:6);
 V1 = [0.1 0; 0 0.1];
 V2 = [2 0; 0 2];
 Ko = lqe(A_lat,B_lat,C_lat,V1,V2);
 case 5 % Version en tiempo discreto
 % tau_min = 1/3.17 modo convergencia en balance
 % aprox. Tm = tau_min/100
 Tm = 0.003;
 I = eye(4);
 F_lat = A_lat*Tm + I;
 G_lat = B_lat*Tm;
 Ko_d = Ko*Tm;
 case 6 % Salir
 break;
 end
end