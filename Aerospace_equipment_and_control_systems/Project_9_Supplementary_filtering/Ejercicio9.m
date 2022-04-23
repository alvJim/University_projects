%---------PRACTICA DE FILTRADO COMPLEMENTARIO ESCA2021---------
% --------------------------GRUPO08----------------------------
%--------------------------------------------------------------
%--------------------------------------------------------------
clear all
%Parametros del S/T
wn = 20; %rad/s
zeta = 0.85; 
Kt = 10/90; %transductor (grados a voltios)
%saturaciones (limites fisicicos)
Vmax = 60; %grados/seg
Xmax = 90; %grados
%Funcion de transferencia del S/T
Gst = tf(Kt*wn^2,[1 2*wn*zeta wn^2]);

%Parametros de los filtros
tauf= 1;
wcf= 20;%rad/s %20
zetaf=1; %1
tauf=1/wcf;
Kf = (1/tauf); %*25;
%Tiempo de muestreo
%Como solo hay una constante de tiempo (tauf)
%Tm = tauf/100;

%Matrices en el espacio de estado del sensor
A = [0 1; -wn^2 -2*zeta*wn];
B = [0; wn^2];
c = [1 0];
D = 0;

%Primer orden
%Filtro wahsout para sensor con deriva
F1 = tf([tauf 0],[tauf 1]);
%filtro paso-bajo para sensor ruidoso
F2 = tf(1,[tauf 1]);
figure(1)
bode(F1,F2);grid
title('Filtrado complementario de primer orden');
legend('Filtro Washout','Filtro Paso-bajo');

%segundo orden
%filtro de washout para sensor con deriva
G1 = tf([1 0 0],[1 2*zetaf*wcf wcf^2]);
%filtro paso-bajo para sensor ruidoso
G2 = tf(wcf^2,[1 2*zetaf*wcf wcf^2]);
figure(2)
bode(G1,G2);grid
title('Filtrado complementario de segundo orden');