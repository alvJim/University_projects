% Práctica : 2 DOF HELO Control Lab:
clear
% Parámetros del modelo
% Máximo voltaje del amplificador: PITCH 24 V y YAW 15 V
VMAX_AMP = [24; 15];
m_heli = 1.3872; % masa del HELO (Kg)
% Posicion del CM respecto del pivote en el eje PITCH (m)
l_cm = 0.1857;
% Momento de inercia equivalente (pitch y yaw) (kg·m^2)
Jp = 0.0384;
Jy = 0.0432;
% Constantes de amortiguamiento viscoso (N.m.s/rad)
Bp = 0.8;
By = 0.318;
% Constantes de Tensión-torque (N.m/V)
Kpp = 0.2041;
Kyy = 0.0720;
Kpy = 0.0068;
Kyp = 0.0219;
% Momentos de inercia respecto del eje de PITCH
JTp = Jp + m_heli*l_cm^2;
JTy = Jy + m_heli*l_cm^2;
% Representacion en el espacio de estados 
A = [0 0 1 0 0 0
    0 0 0 1 0 0
    0 0 -Bp/(JTp+m_heli*l_cm^2) 0 0 0
    0 0 0 -By/(JTp+m_heli*l_cm^2) 0 0
    1 0 0 0 0 0
    0 1 0 0 0 0]; %6*6
B = [0 0 
    0 0 
    Kpp/(JTp+m_heli*l_cm^2) Kpy/(JTp+m_heli*l_cm^2) 
    Kyp/(JTy+m_heli*l_cm^2) Kyy/(JTy+m_heli*l_cm^2) 
    0 0
    0 0]; %6*4
C = [1 0 0 0 0 0  
    0 1 0 0 0 0]; %2*6
D = zeros(2,2); %3*4
% Condiciones iniciales x = [ p y p_dot y_dot int_p int_y]
x0 = [-40.5*pi/180 0 0 0 0 0]';
% Dinámica del modelo en lazo abierto
disp('Sistema en lazo abierto');
helo_la = ss(A,B,C,D)
disp('autovalores lazo abierto');
eig(helo_la)
disp('FTLA:');
G_helo_la = tf(helo_la)
disp('polos LA');
pole(G_helo_la);
damp(G_helo_la);
% matriz de controlabilidad
Mc = ctrb(helo_la)
if rank(Mc)==length(A)
disp('Sistema controlable');
else
disp('Sistema no controlable');
end
% Calculo matriz de realimentacion de estados K (LQR)
Q = diag([100 100 100 100 100 100]);
R = diag([0.5 0.5]);
Kc = lqr(A,B,Q,R);
% Ganancia del controlador feedforward: uff = Kff*cos(p_d)
g = 9.81; % m/s^2
Kff = [m_heli*g*l_cm/Kpp; 0];
% Dinámica del modelo en lazo cerrado
disp('Sistema en lazo cerrado');
hover_lc = ss(A-B*Kc,B,C,D)
disp('autovalores lazo cerrado');
eig(hover_lc)
disp('FTLC:');
G_hover_lc = tf(hover_lc)
disp('polos LC');
pole(G_hover_lc);
damp(G_hover_lc);

% Pr�ctica de control del helic�ptero (2D-Helo)
% ESCA/ASAN - Prof. Luis Garc�a  NOV-21
% Imprime los resultados de las pruebas.
% 
% Se carga en el workspace el fichero de datos:
load('data_Helo.mat');

% Imprimir datos de pitch, yaw, pitch rate, yaw rate:
figure(1)
plot(data_theta(:,1),data_theta(:,2),data_theta(:,1),data_theta(:,3),'linewidth',2); grid;
title('Respuesta a SP onda cuadrada');
xlabel('Tiempo (seg)');
ylabel('Angulo de cabeceo (grados)');
legend('Referencia','Respuesta');
figure(2)
plot(data_psi(:,1),data_psi(:,2),data_psi(:,1),data_psi(:,3),'linewidth',2); grid;
title('Respuesta a SP onda cuadrada');
xlabel('Tiempo (seg)');
ylabel('Angulo de gui�ada (grados)');
legend('Referencia','Respuesta');
figure(3)
plot(data_thetadot(:,1),data_thetadot(:,2),'linewidth',2); grid;
title('Respuesta a SP onda cuadrada');
xlabel('Tiempo (seg)');
ylabel('Velocidad angular cabeceo (grados/s)');
figure(4)
plot(data_psidot(:,1),data_psidot(:,2),'linewidth',2); grid;
title('Respuesta a SP onda cuadrada');
xlabel('Tiempo (seg)');
ylabel('Velocidad angular gui�ada (grados/s)');

% Imprimir datos de tension aplicada a motores
figure(5)
plot(data_Vm(:,1),data_Vm(:,2),data_Vm(:,1),data_Vm(:,3),'linewidth',2); grid;
title('Tension aplicada rotores');
xlabel('Tiempo (seg)');
ylabel('[u_p u_y] (voltios)');
legend('Rotor principal','Rotor de cola');