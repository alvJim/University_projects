% -----------------------------------------------------------------------
% Ficheros: E1_VehiculoAereo1DOF.m
% E1_VehiculoAereo1DOFsim.mdl
%
% Fecha: 20 de septiembre de 2021
% Asignatura: ESCA
%
% Par�metros de veh�culo a�reo 1-DOF
% -----------------------------------------------------------------------
clear
%
% Din�mica veh�culo (1-DOF):
% m*Vdot(t) = Fp(t) - FB(t); donde FB(t) = B*V^2(t)
m = 400; % Kg
B = 0.162; % N/(m/s) Coeficiente de fricci�n viscosa
Vnom = 800*10/36; % 800 Km/h en m/s, Velocidad nominal
% Actuador (Sistema de propulsi�n):
% taup*Fpdot(t) + Fp(t) = KT*CO(t), donde CO(t) = COnom + u(t)
Fpnom = 8e3; % N
Fplim = [0 1]*16e3; % N
Fpdotlim = [-1 1]*16e3/5; % N/s
taup = 1; % s
Kt = 16e3/10; % N/Volt
% Sensor/Transductor: Sistema de primer orden:
% taum*Vmdot(t) + Vm(t) = Km*V(t)
taum = 0.1; % s
Km = 10/300; % Volt/(m/s)
Vmlim = [0 10]; % Volt
Vmnom = Vnom*Km; % Volt
% Filtro pasa-bajas de primer orden:
% tauf*Vmfdot(t) + Vmf(t) = Vm(t)
tauf = 0.02; % s ajustable
Vmfnom = Vmnom; % Volt
% Controlador, ejemplo P, PI, PD, PID
% CO(t) = COnom + u(t), donde u(t) es la salida del controlador
COnom = Fpnom/Kt; % Volt
COlim = [0 10]; % Volt
% Estabilidad en lazo abierto
% Linealizaci�n del sistema (mediante jacobianos).
% Vector de estados X = (V Fp Vm Vmf)'
% Vector de control U = CO
% Vector de medidas Y = Vmf = X4
Ap = [-2*sqrt(B*Fpnom)/m 1/m 0 0
 0 -1/taup 0 0
 Km/taum 0 -1/taum 0
 0 0 1/tauf -1/tauf];
Bp = [0 Kt/taup 0 0]';
Cp = [0 0 0 1];
Dp = 0;
ss_sys = ss(Ap,Bp,Cp,Dp,'Statename',{'V','Fp','Vm','Vmf'}, ...
    'Inputname',{'CO'}, ...
 'Outputname',{'Vmf'})
% PID para Simulink (valores aprox)
Kp = 5;
Ki = 0.5;
Kd = 5;
taufd = Kd/10;
% Parametros de simulaci�n
modeloSim = 'E1_VehiculoAereo1DOFsim';
open_system(modeloSim);
dt = 0.002; % paso de integraci�n (aprox tau_min/10)
while 1
 opt = menu(' E1_VehiculoAereo1DOF', ...
 ' Estabilidad en lazo abierto', ...
 'Dise�o controlador proporcional (Kp)', ...
 'Simulaci�n: Presentaci�n de resultados usando scopes', ...
 'Simulaci�n: Presentaci�n de resultados usando bloques workspace', ...
 'Salir');
 switch opt
 case 1 % Estabilidad en lazo abierto
 eig(ss_sys.a)
 G_sys = tf(ss_sys)
 zpk(G_sys)
 damp(G_sys)
 dcgain(G_sys)
 step(G_sys)
 case 2 % Dise�o controlador proporcional (Kp)
 rlocus(G_sys); grid;
 K = input('Introduzca ganancia del controlador (Kp): ');
 Glc_sys = minreal(K*G_sys/(1 + K*G_sys));
 figure(1)
 zpk(Glc_sys)
 damp(Glc_sys)
 dcgain(Glc_sys)
 step(G_sys,Glc_sys); grid;
 case 3 % Presentaci�n de resultados usando scopes
 sim(modeloSim);
 t = dataScope_V(:,1);
 Vsp = dataScope_V(:,2);
 V = dataScope_V(:,3);
 plot(t,Vsp,t,V,'linewidth',2); grid
 axis([0 120 790 860]);
 xlabel('Tiempo (s)');
 ylabel('Velocidad (Km/h)');
 title('Respuesta Escal�n de 50 Km/s y perturbaci�n de 1000 N');
 case 4 % Presentaci�n de resultados usando bloques workspace
 sim(modeloSim);
 figure(2)
 subplot(3,1,1)
 t = dat_t;
 Vsp = 3.6*dat_Vsp;
 V = 3.6*dat_V;
 plot(t,Vsp,t,V,'linewidth',2); grid
 ylabel('Velocidad (Km/h)');
 title('Respuesta Escal�n de 50 Km/s y perturbaci�n de 1000 N');
 subplot(3,1,2)
 CO = dat_CO;
 plot(t,CO,'linewidth',2); grid
 ylabel('CO(t)(Volt)');
 title('Se�al de control');
 subplot(3,1,3)
 Fp = dat_Fp;
 plot(t,Fp,'linewidth',2); grid
 xlabel('Tiempo (s)');
 ylabel('Fp(t)(N)');
 title('Empuje del propulsor (N)');
 case 5 % Salir
 break;
 end
end