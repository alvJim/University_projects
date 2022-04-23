clear all
clc
% Parámetros del modelo
CMD_RATE_LIMIT = 60*pi/180; % Lim de velocidad SP
VMAX_AMP = 24; % Máxima tensión de salida (V)
Kf = 0.1188 ; % Constante de empuje (N/V)
Kt = 0.0036 ; % Constante de torque (N·m/V)
% Momentos de inercia alrededor de los ejes (kg.m^2)
Jp = 0.0552 ;
Jy = 0.1100 ;
Jr = 0.0552 ;
L = 0.197 ; % Distancia desde el pivote a cada motor (m)
% Representación en el espacio de estados

A = [   0   0   0   1   0   0;
        0   0   0   0   1   0;
        0   0   0   0   0   1;
        0   0   0   0   0   0;
        0   0   0   0   0   0;
        0   0   0   0   0   0]; % dim 6×6

B = [   0           0           0           0;
        0           0           0           0;
        0           0           0           0;
        -(Kt/Jy)    -(Kt/Jy)    (Kt/Jy)     (Kt/Jy);
        (L*Kf/Jp)   -(L*Kf/Jp)  0           0; 
        0           0           (L*Kf/Jp)   -(L*Kf/Jp)]; % dim 6×4

C = [1  0   0   0   0   0;
     0  1   0   0   0   0;
     0  0   1   0   0   0]; % dim 3×6

D = zeros(3,4); % dim 3×4;
% Dinámica del modelo en lazo abierto

dt = 0.002;

disp('Sistema en lazo abierto');
hover_la = ss(A,B,C,D)
disp('autovalores lazo abierto');
eig(hover_la)
disp('FTLA:');
G_hover_la = tf(hover_la)
disp('polos LA');
pole(G_hover_la);
damp(G_hover_la);

% matriz de controlabilidad
T = ctrb(hover_la)
if rank(T)==length(A)
disp('Sistema controlable');
else
disp('Sistema incontrolable');
end
% Diseño controlador LQR
% Calculo matriz de realimentación de estados K
% x = [ y p r ydot pdot rdot]
Q = diag([100 100 100 10 10 10]);
R = 0.05*diag([1 1 1 1]);
K = lqr(A,B,Q,R);
V_bias = 2.0; % Voltaje Bias aplicado a los motores (V)
% Dinámica del modelo en lazo cerrado
disp('Sistema en lazo cerrado');
hover_lc = ss(A-B*K,B,C,D)
disp('autovalores lazo cerrado');
eig(hover_lc)
disp('FTLC:');
G_hover_lc = tf(hover_lc)
disp('polos LC');
pole(G_hover_lc);
damp(G_hover_lc);