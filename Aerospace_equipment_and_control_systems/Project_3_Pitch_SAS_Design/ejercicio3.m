% -------------------------------------------------------------------------
%   Ficheros: E3_PitchSAS.m
%             E3_PitchSASsim.mdl
%
%   Fecha: 12 de octubre de 2021
%   Asignatura: ESCA
%
%   Dise?o de PITCH SAS (PITCH DAMPER) para F-16
% -------------------------------------------------------------------------
clear
% Efectos de la realimentaci?n de la velocidad angular de cabeceo (q) y el 
% angulo de ataque (alfa)
disp('F16 vuelo recto y nivelado (h=0 ft Vel=500 ft/s)');
%        VT           alfa        theta        q
A = [-1.9311e-02  8.8157e+00 -3.2170e+01 -5.7499e-01;
      -2.5389e-04 -1.0189e+00  0.0000e+00  9.0506e-01;
       0.0000e+00  0.0000e+00  0.0000e+00  1.0000e+00;
       2.9465e-12  8.2225e-01  0.0000e+00 -1.0774e+00];
B = [ 1.7370e-01 -2.1499e-03 0.0000e+00 -1.7555e-01]';% delta_e
C = [ 0.000000e+00 5.729578e+01 0.000000e+00 0.000000e+00; % alfa 
      0.000000e+00 0.000000e+00 0.000000e+00 5.729578e+01];% q
D = 0; 
SN = {'VT','alfa','theta','q'};
IN = {'delta_e'};
ON = {'alfa_m','q_m'};
sys = ss(A,B,C,D,'StateName',SN,'InputName',IN,'OutputName',ON)
while(1)
    opt = menu(' E3 Pitch SAS', ...
          'An?lisis en lazo abierto', ...
          'Dise?o controlador Kalfa (lazo de AOA)', ... 
          'Dise?o controlador Kq (lazo de Pitch rate)', ... 
          'Presentaci?n de resultados', ...
          'Salir');
  
    switch opt
        case 1 % An?lisis en lazo abierto
            eig(sys)
            zpk(sys)
            damp(sys)
            % Funci?n de transferencia entre el elevador y el ?ngulo de ataque
            % alpha(s)/delta_e(s):
            G_alpha_delta_e = zpk(sys(1,1))
            % que es equivalente a:
            G_alpha_delta_e = zpk(ss(A,B,C(1,:),D))
            
            %Actuador(servo + superficie de control
            tau_a = 1/20.2;
            delta_e_lim = [-30 30]*pi/180;% l?mites de posici?n (rad)
            delta_e_dot_lim = [-60 60]*pi/180;% l?mites de velocidad (rad/s)
            % Filtro pasobajo de alpha
            tau_F = 0.1;
            % Planta ampliada con el filtro de AOA y el actuador:
            %         VT   alfa   theta   q    x_a        x_f
            A_aug = [           A          B        zeros(4,1);
                             zeros(1,4)   -1/tau_a  0;
                             0 1/tau_F 0 0  0      -1/tau_F];
            B_aug = [0 0 0 0 1/tau_a 0]';% ue
            % alfa  q  alfa_F
            C_aug = [    C        zeros(2);
                     zeros(1,4)  0  180/pi];

            D_aug = 0;
            SN = {'VT','alfa','theta','q','x_a','x_F'}; IN = {'u_e'};
            ON = {'alfa_m','q_m','alfa_F'};
            sys_aug = ss(A_aug,B_aug,C_aug,D_aug,...
                                 'StateName',SN,'InputName',IN,'OutputName',ON)
            zpk(sys_aug)
        case 2 % Dise?o controlador Kalfa (lazo de AOA)
            % Funci?n de transferencia entre la se?al de control u_e y alpha_F:
            G_alphaF_ue = zpk(sys_aug(3,1))
            % Obtenemos el lugar de las ra?ces:
            rlocus(-G_alphaF_ue); grid% Se ha cambiado de signo para no tener en
            % cuenta en dise?o la respuesta inversa
            axis([-20 1 -10 10]);
            
            % para Kalfa = 0.5 el polo corto periodo es -0.70+/-j2.0 (wn=2.2 rad/s
            % zeta = 0.33) wn aceptable pero zeta peque?o (poco amortiguado).
            Kalfa = input('Introduzca ganancia del controlador (Kalfa):' ); 
            if isempty(Kalfa)
                Kalfa = -0.5;
            end
            % cerramos el lazo para alfa_F e incluimos la ganancia Kalfa: 
            sys_aug_lc = feedback(sys_aug,Kalfa,1,3)% se escoge entrada 1 (u_e
            % y salida 3 (alpha_F)
            % Manualmente ser?a:
            Alc = A_aug - B_aug*Kalfa*C_aug(3,:);
            sys_aug_lc = ss(Alc,B_aug,C_aug,0)% q/u T.F
            zpk(sys_aug_lc)
            damp(sys_aug_lc)
        case 3 % Dise?o controlador Kq (lazo de Pitch rate)
            G_q_u = zpk(sys_aug_lc(2,1));
            rlocus(-G_q_u); grid;
            % para Kq = 0.25 el polo corto periodo es -2.02+/-j1.94 (wn=2.8 rad/s
            % zeta = 0.72) ambos aceptables.
            Kq = input('Introduzca ganancia del controlador (Kq): '); 
            if isempty(Kq)
                Kq = -0.25;
            end
            % cerramos el lazo para alfa y q e incluimos las ganancias Kalfa y Kq:
            sys_aug_lc = feedback(sys_aug,[Kq Kalfa],1,[2 3])
            zpk(sys_aug_lc)
            damp(sys_aug_lc)
            figure(1)
            step(sys_aug_lc); grid
            figure(2)
            impulse(sys_aug_lc); grid
       case 4 % Presentaci?n de resultados
              % tailplane doublet 1 s & amplitud 1:
              t= 0:0.002:8;
              u= zeros(1,length(t));
              u(101:200)=1;
              figure(3)
              lsim(sys_aug,sys_aug_lc,u,t); grid;
              % Alternativamente para mayor control de los gr?ficos:
                y1 = lsim(sys_aug,u,t);
                y2 = lsim(sys_aug_lc,u,t);
              figure(4)
              subplot(311) 
              plot(t,u,t,y1(:,1),t,y2(:,1),'linewidth',2); grid 
              ylabel('\alpha');
              subplot(312) 
              plot(t,u,t,y1(:,2),t,y2(:,2),'linewidth',2); grid 
              ylabel('q');
              subplot(313) 
              plot(t,u,t,y1(:,3),t,y2(:,3),'linewidth',2); grid 
              ylabel('\alpha_F');
              xlabel('Tiempo (s)');
              legend('Doublet','sin SAS','con Pitch SAS');
        case 5 % Salir 
            break; 
            
    end
end
  
            
            
            
            
            
            
            
  
  
  
  