% -------------------------------------------------------------------------
% Fichero: ejercicio2lateral.m
%
% Fecha: 01 de octubre de 2021
% Asignatura: ESCA
%
% Dise�o de un controlador para la din�mica lateral de vuelo de un
% avi�n de combate F16.
% -------------------------------------------------------------------------
clear
%{
 ESTADOS
 beta �ngulo de deriva (sideslip angle)
 phi �ngulo de balance o alabeo (roll/bank angle)
 p velocidad angular de balance (roll rate)
 r velocidad angular de gui�ada (yaw rate)
 ACTUADORES
 delta_a �ngulo de desviaci�n de alerones (aileron)
 delta_r �ngulo de desviaci�n del tim�n de direcci�n (rudder)
 MEDIDAS (SENSORES)
 beta_m �ngulo de deriva (sideslip angle)
 phi_m �ngulo de balance o alabeo (roll/bank angle)
 p_m velocidad angular de balance (roll rate)
 r_m velocidad angular de gui�ada (yaw rate)
%}
% Matrices (A,B,C,D) del sistema:
% beta phi p r
A_lat = [-0.2723 0.0640 0.0469 -0.9918
 0 0 1 0.0470
 -28.2010 0 -3.2574 0.6348
 7.2278 0 -0.0335 -0.4542];
% delta_a delta_r
B_lat = [ 0.0002 0.0007
 0 0
 -0.6222 0.1152
 -0.0268 -0.0584];
C_lat = eye(4);
D_lat = zeros(4,2);
ss_lat = ss(A_lat,B_lat,C_lat,D_lat,'Statename',{'beta','phi','p','r'}, ...
 'Inputname',{'delta_a','delta_r'}, ...
 'Outputname',{'beta_m','phi_m','p_m','r_m'});
G_lat = tf(ss_lat);
% a) Determinar las F. de T que relacionan las diferentes variables de
% estado con cada una de las variables manipuladas
% beta_m / delta_a
Gbetam_deltaa = G_lat(1,1);
% Otro m�todo:
A = A_lat;
B = B_lat(:,1);
C = [1 0 0 0];
D = 0;
ss_betam_deltaa = ss(A,B,C,D,'Statename',{'beta','phi','p','r'}, ...
 'Inputname',{'delta_a'}, ...
 'Outputname',{'beta_m'});
G_betam_deltaa = tf(ss_betam_deltaa);
% o tambi�n:
[numeradorG,denominadorG] = tfdata(ss_betam_deltaa);
G_betam_deltaa = tf(numeradorG,denominadorG);
% phi_m / delta_a
G_phim_deltaa = G_lat(2,1);
ss_phim_deltaa = ss(A_lat,B_lat(:,1),[0 1 0 0],0, ...
 'Statename',{'beta','phi','p','r'}, ...
 'Inputname',{'delta_a'}, ...
 'Outputname',{'phi_m'});
G_phim_deltaa = tf(ss_phim_deltaa);
% b) Calcular los ceros y polos del sistema (modos caracter�sticos)
% FT en t�rminos de polos y ceros
zpk(G_phim_deltaa)
% par�metros de los polos del sistema
damp(G_phim_deltaa)
% Ganancia estacionaria
dcgain(G_phim_deltaa)
% Comprobaci�n de la equivalencia entre autovalores y polos
eig(A_lat)
% Otras FT:
Gpm_deltaa = G_lat(3,1); % p_m / delta_a
Grm_deltaa = G_lat(4,1); % r_m / delta_a
Gbetam_deltar = G_lat(1,2); % beta_m / delta_r
Gphim_deltar = G_lat(2,2); % phi_m / delta_r
Gpm_deltar = G_lat(3,2); % p_m / delta_r
Grm_deltar = G_lat(4,2); % r_m / delta_r
%-----------------------------------------------------------
% c) Determinar los t�rminos exponenciales asociados, as� como la constante de
% tiempo asociada a cada polo
% Nota: constante de tiempo efectiva SSO: tau_ef = 1/(zeta*wn)
%{
zpk(Gbetam_deltaa)
Zero/pole/gain from input "delta_a" to output "beta_m":
 0.0002 (s-2.097) (s^2 - 7.196s + 46.19)
---------------------------------------------
(s+3.171) (s+0.01696) (s^2 + 0.7955s + 8.412)
damp(Gbetam_deltaa)
 Eigenvalue Damping Freq. (rad/s)

 -1.70e-002 1.00e+000 1.70e-002 ESPIRAL (+lento)
 -3.98e-001 + 2.87e+000i 1.37e-001 2.90e+000 DUTCH ROLL
 -3.98e-001 - 2.87e+000i 1.37e-001 2.90e+000
 -3.17e+000 1.00e+000 3.17e+000 CONV. EN BALANCE
%}
% Se obtiene los t�rminos exponenciales de cada modo mediante c�lculo
% simb�lico:
syms s
ilaplace(1/(s + 0.01696)) % ESPIRAL
ilaplace(1/(s^2 + 0.7955*s + 8.412)) % DUTCH ROLL
ilaplace(1/(s + 3.171)) % CONV. EN BALANCE