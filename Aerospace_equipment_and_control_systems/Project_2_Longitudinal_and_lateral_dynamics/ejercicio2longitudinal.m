% -------------------------------------------------------------------------
% Fichero: ejercicio2longitudinal.m
%
% Fecha: 01 de octubre de 2021
% Asignatura: ESCA
%
% Dise�o de un controlador para la din�micalongitudinal
% -------------------------------------------------------------------------

A_lon = [-0.0191 2.5172 -9.8036 -0.2540
 -0.0008 -0.9713 0 0.9126
 0 0 0 1
 0 -0.7234 0 -1.2607];
B_lon = [ 0.1128 0.0298
 0 -0.0022
 0 0
 0 -0.1743];
C_lon = eye(4);
D_lon = zeros(4,2); %para definir las nuevas matrices
ss_lon = ss(A_lon,B_lon,C_lon,D_lon,'Statename',{'vt','alpha','theta','q'}, ...
 'Inputname',{'delta_th','delta_e'}, ...
 'Outputname',{'vt_m','alpha_m','theta_m','q_m'});
G_lon = tf(ss_lon); 
G_alpham_deltath = G_lon(2,1);
%apartado b
zpk(G_alpham_deltath)
damp(G_alpham_deltath)
dcgain(G_alpham_deltath)
eig(A_lon)
%apartado c 
syms s
ilaplace(1/(s^2 + 0.01693*s + 0.003004)) %FUGOIDE
ilaplace(1/(s^2 + 2.234*s + 1.889)) %CORTO PERIODO
