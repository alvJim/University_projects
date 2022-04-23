% -----------------------------------------------------------------------
% Ficheros: E6_ControlLongitudinalMIMO.m
% E6_ControlLongitudinalMIMOsim.mdl
%
% Fecha: 20 de octubre de 2021
% Asignatura: ESCA
% -----------------------------------------------------------------------
clear
dt =0.001;
%1. La din�mica de una aeronave se ha linealizado para una condici�n de vuelo
% recto y nivelado (1000 m y 153 m/s). Las matrices que corresponden al modelo
% linealizado de la din�mica lateral se indican a continuaci�n.
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
C_lon = [1 0 0 0 % vT
 0 0 180/pi 0]; % theta
D_lon = zeros(2);
SN = {'vT','alfa','theta','q'};
IN = {'delta_TH','delta_e'};
ON = {'vT','theta'};
sys_lon = ss(A_lon,B_lon,C_lon,D_lon,'StateName',SN,'InputName',IN,'OutputName',ON)
% Actuadores para simulaci�n
% Elevador
tau_e = 1/20;
delta_e_lim = [-25 25]; % l�mites de posici�n (deg)
delta_e_dot_lim = [-90 90]; % l�mites de velocidad (deg/s)
% Throttle
tau_th = 0.5;
delta_th_lim = [-0.2 0.8]*100; % l�mites (%)
delta_th_dot_lim = [-0.24 0.24]*100; % l�mites de velocidad (%/s)
%--------------------------------------------------------------------------
% Analisis en lazo abierto (sin actuadores)
zpk(sys_lon)
damp(sys_lon)
% Controlabilidad y Observabilidad:
Mc = [B_lon A_lon*B_lon A_lon^2*B_lon A_lon^3*B_lon];
% o con la funci�n de Matlab:
Mc = ctrb(A_lon,B_lon);
if rank(Mc)==length(A_lon)
 disp('** Sistema controlable');
else
 disp('** Sistema no contrable');
end
Mo = [C_lon; C_lon*A_lon; C_lon*A_lon^2; C_lon*A_lon^3];
% o con la funci�n d Matlab:
Mo = obsv(A_lon,C_lon);
if rank(Mo)==length(A_lon)
 disp('** Sistema observable');
else
 disp('** Sistema no observable');
end
%{
CRITERIOS DE DISE�O
1. Al dise�ar un controlador, obs�rvese que si los polos dominantes del controlador
 se situan alejados a la izquierda del eje imaginario, los elementos de la matriz
 de ganancia de realimentaci�n del estado Kc se har�n grandes. Grandes valores de
 la ganancia Kc har�n que la salida del actuador sea tambi�n grande, de manera que
 puede dar lugar a saturaci�n de los actuadores.
2. Tambi�n, al situar los polos del observador suficientemente alejados a la izquierda
 del eje imaginario, el conjunto controlador-observador puede hacerse inestable,
 aunque el sistema en lazo cerrado sea estable.
3. Si el controlador-observador se hace inestable, se mueven los polos del observador
 a la derecha en el semiplano izquierdo del plano s hasta que se estabilice.
Tambi�n,
 la localizaci�n de los polos en lazo cerrado deseados puede tener que modificarse.
4. Obs�rvese que si los polos del observador se colocan lejos a la izquierda del eje
 imaginario, el ancho de banda del observador aumentar� y originar� problemas de
 ruido. El requisito general es que el ancho de banda deber�a ser suficientemente
 bajo para que el ruido de los sensores que miden y(t) y se realimenta al observador
 no sea un inconveniente.
En resumen, los polos del observador deben ser bastante m�s r�pidos que los de la
planta en lazo cerrado, pero no tanto como para que generen problemas de inestabilidad
y ruido.
Como regla general, los polos del observador deben ser de dos a cinco veces m�s
r�pidos
que los polos del controlador para asegurarse que el error de observaci�n (error de
estimaci�n) converge a cero r�pidamente.
%}
while 1
 opt = menu('ESCA: Ejercicio 6 Control longitudinal',...
 '1. Control modal - Observador de Luenberger',...
 '2. Dise�o LQG (LQR + Filtro de Kalman)',...
 '3. Acci�n integral: Control Modal + Filtro de Kalman',...
 '4. Acci�n integral: LQG (LQR + Filtro de Kalman)',...
 '5. Version en tiempo discreto',...
 '6. Salir');
 switch opt
 case 1 % a) Control modal - Observador de Luenberger
 polos_c = [-1 -3 -4 -8]';
 Kc = place(A_lon,B_lon,polos_c);
 Ayr = A_lon - B_lon*Kc;
 Cyr = C_lon - D_lon*Kc;
 PreC=[Cyr*(-Ayr)^(-1)*B_lon+D_lon]^(-1);
 Byr = B_lon*PreC;
 Dyr = D_lon*PreC;
 sys_lon_lc = ss(Ayr,Byr,Cyr,Dyr)
 pole(sys_lon_lc)
 zero(sys_lon_lc)
 dcgain(sys_lon_lc)
 zpk(sys_lon_lc)
 damp(sys_lon_lc)
 step(sys_lon_lc); grid

 polos_o = polos_c*2;
 Ko = place(A_lon',C_lon',polos_o);
 Ko = Ko';
 case 2 % Dise�o LQG (LQR + Filtro de Kalman)
 V1 = [0.1 0; 0 0.1];
 V2 = [2 0; 0 2];
 Ko = lqe(A_lon,B_lon,C_lon,V1,V2);
 q1 = 5e7; % vT
 q2 = 5e7; % alfa
 q3 = 5e7; % theta
 q4 = 5e7; % q
 r1 = 1e4; % dTH
 r2 = 1e4; % de
 Qc = [q1 0 0 0;0 q2 0 0;0 0 q3 0;0 0 0 q4];
 Rc = [r1 0; 0 r2];
 Kc = lqr(A_lon,B_lon,Qc,Rc);
 Ayr = A_lon - B_lon*Kc;
 Cyr = C_lon - D_lon*Kc;
 PreC = [Cyr*(-Ayr)^(-1)*B_lon+D_lon]^(-1);
 case 3 % Acci�n integral: Control Modal + Filtro de Kalman
 A_lon_a = [A_lon zeros(4,2);-C_lon zeros(2,2)];
 B_lon_a = [B_lon;-D_lon];
 polos_a = [-1 -1.5 -2 -2.5 -3 -4];
 V1 = [0.1 0; 0 0.1];
 V2 = [2 0; 0 2];
 Ko = lqe(A_lon,B_lon,C_lon,V1,V2);
 Kc_a = place(A_lon_a,B_lon_a,polos_a);
 Kc = Kc_a(:,1:4);
 Ki = -Kc_a(:,5:6);
 case 4 % Acci�n integral: LQG (LQR + Filtro de Kalman)
 A_lon_a = [A_lon zeros(4,2);-C_lon zeros(2,2)];
 B_lon_a = [B_lon;-D_lon];
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
 Kc_a = lqr(A_lon_a,B_lon_a,Qc,Rc);
 Kc = Kc_a(:,1:4);
 Ki = -Kc_a(:,5:6);
 V1 = [0.1 0; 0 0.1];
 V2 = [2 0; 0 2];
 Ko = lqe(A_lon,B_lon,C_lon,V1,V2);
 case 5 % Version en tiempo discreto
     % tau_min = 1/1.37 modo corto periodo
 % aprox. Tm = tau_min/100
 Tm = 0.003;
 I = eye(4);
 F_lon = A_lon*Tm + I;
 G_lon = B_lon*Tm;
 Ko_d = Ko*Tm;
 case 6 % Salir
 break;
 end
end