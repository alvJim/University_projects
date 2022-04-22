% -----------------------------------------------------------------------
%   Ficheros: E4_RollYawDamper.m
%             E4_LateralSASsim.mdl
%
%   Fecha: 1 de octubre de 2021
%   Asignatura: ESCA
%
% Diseño de ROLL/YAW DAMPER para F-16
% ----------------------------------------------------------------------- 
clear
% Lateral-Direccional Stability Augmentation
disp('F16 vuelo recto y nivelado (h=0 ft Vel=205 ft/s)');
% beta phi psi pr
A = [-0.13150 0.14858 0.0 0.32434 -0.93964;
      0.0     0.0     0.0   1.0    0.33976;
      0.0     0.0     0.0   0.0     1.0561;
     -10.614  0.0     0.0  -1.1793  1.0023;
     0.99655  0.0     0.0 -0.0018174 -0.25855];
%      delta_a     delta_r
B = [ 0.00012049 0.00032897;
      0.0        0.0;
      0.0        0.0;
     -0.1031578  0.020987;
     -0.0021330 -0.010715];
C = [0.0 0.0 0.0 180/pi 0.0; %p
    0.0 0.0 0.0 0.0 180/pi];% r
D = zeros(2,2);
SN = {'beta','phi','psi','p','r'};
IN = {'delta_a','delta_r'};
ON = {'p','r'};
sys = ss(A,B,C,D,'StateName',SN,'InputName',IN,'OutputName',ON)

while(1)
opt = menu(' E4 Roll/Yaw Damper SAS', ...
'Análisis en lazo abierto', ... 
'Diseño amortiguador de balance', ... 
'Diseño amortiguador de guiñada', ... 
'Presentación de resultados', ... 
'Comparativa de varios diseños', ... 
'Salir');
switch opt
    case 1 % Análisis en lazo abierto
            lambda = eig(sys)
            zpk(sys)
            damp(sys)
            % Sistema ampliado con el filtro washout y los actuadores:
            tau_a = 1/20.2;
            tau_r = 1/20.2;
            tau_W = 1;
            % se suprime psi del sistema
            A(3,:) = []; A(:,3) = []; B(3,:) = []; C(:,3)=[];
%           beta phi p  r   delta_a delta_r  x_W
            A_aug = [  A         B        zeros(4,1); 
                     0 0 0 0  -1/tau_a   0      0;
                     0 0 0 0      0    -1/tau_r 0;
                     0 0 0 180/pi 0       0    -1/tau_W]; 
            B_aug = [0 0 0 0 1/tau_a  0      0; % ua
                     0 0 0 0   0     1/tau_r 0]';% ur
            % p r_W
            C_aug = [C [0 0 0; 0 0 -1/tau_W]];
            D_aug = D;
            SN = {'beta','phi','p','r','delta_a','delta_r','x_W'};
            IN = {'u_a','u_r'};
            ON = {'p','r_W'};
            sys_aug = ss(A_aug,B_aug,C_aug,D_aug','StateName',SN,'InputName',IN,'OutputName',ON) 
            zpk(sys_aug)
case 2 % Diseño amortiguador de balance 
    G_p_ua = zpk(sys_aug(1,1)) 
    rlocus(-G_p_ua); grid;
    axis([-12 1 -5 5]);
    Kp = input('Introduzca ganancia del controlador (Kp): '); 
    if isempty(Kp)
        Kp = -0.2;
    end
    Kr = 0.0;
    % cerramos el lazo de velocidad de balance p:
    K = [Kp 0; 0 Kr];
    disp('cerramos el lazo de velocidad de balance p'); 
    sys_aug_lc = minreal(feedback(sys_aug,K)) 
    zpk(sys_aug_lc)
    damp(sys_aug_lc)
case 3 % Diseño amortiguador de guiñada 
    G_rW_ur = zpk(sys_aug_lc(2,2)) 
    rlocus(-G_rW_ur); grid;
    axis([-12 1 -5 5]);
    Kr = input('Introduzca ganancia del controlador (Kr): '); 
    if isempty(Kr)
        Kr = -3.5;
    end
    K = [Kp 0; 0 Kr];
    % cerramos el lazo de velocidad de guiñada q:
    disp('cerramos el lazo de velocidad de guiñada q');
    sys_aug_lc = minreal(feedback(sys_aug,K)) 
    zpk(sys_aug_lc)
    damp(sys_aug_lc)
case 4 % Presentación de resultados 
    %p
    figure(1)
    subplot(211) 
    impulse(sys_aug(1,1),sys_aug_lc(1,1),10); grid; 
    legend('sin SAS','con Roll/Yaw damper');
    % r_W
    subplot(212)
    impulse(sys_aug(2,2),sys_aug_lc(2,2),10); grid;
    % doublet 1s & amplitude 1:
    t= 0:0.01:10;
    u= zeros(1,length(t));
    u(101:200)=1;
    %p
    figure(2)
    subplot(211) 
    lsim(sys_aug(1,1),sys_aug_lc(1,1),u,t); grid; 
    title('Pulso en lazo de balance');
    legend('sin SAS','con Roll/Yaw damper');
    % r_W
    subplot(212) 
    lsim(sys_aug(2,2),sys_aug_lc(2,2),u,t); grid; 
    title('Pulso en lazo de guiñada');
    % p y r_W from u_a
    figure(3) 
    lsim(sys_aug(:,1),sys_aug_lc(:,1),u,t); grid; 
    title('Pulso en lazo de balance');
    legend('sin SAS','con Roll/Yaw damper');
    % p y r_W from u_r
    figure(4) 
    lsim(sys_aug(:,2),sys_aug_lc(:,2),u,t); grid; 
    title('Pulso en lazo de guiñada');
    legend('sin SAS','con Roll/Yaw damper');
case 5 % Comparativa de varios diseños
      % Respuesta con otros valores de la matriz K:
    Kp = -0.4;
    Kr = -1.3;
    K = [Kp 0; 0 Kr];
    % cerramos el lazo de velocidad de guiñada q:
    sys_aug_lc2 = minreal(feedback(sys_aug,K));
    figure(5)
    subplot(211) 
    lsim(sys_aug(1,1),sys_aug_lc(1,1),sys_aug_lc2(1,1),u,t); grid; 
    title('Pulso en lazo de balance');
    legend('sin SAS','con Roll/Yaw damper (Kp=-0.2, Kr=-3.5', ... 
        'con Roll/Yaw damper (Kp=-0.4, Kr=-1.3');
    subplot(212) 
    lsim(sys_aug(2,2),sys_aug_lc(2,2),sys_aug_lc2(2,2),u,t); grid; 
    title('Pulso en lazo de guiñada');
case 6 % Salir 
    break;
end
end