% Práctica de control del sustentador (3D-Hover)
% Imprime los resultados de las pruebas.
%
% Se carga en el workspace el fichero de datos:
load('data_Hover.mat');
% Imprimir datos de yaw, pitch roll
figure(1)
subplot(311)
plot(data_yaw(:,1),data_yaw(:,2),data_yaw(:,1),data_yaw(:,3),'linewidth',2); grid;
title('Respuesta a SP onda cuadrada');
xlabel('Tiempo (seg)');
ylabel('Guiñada (grados)');
legend('Referencia','Respuesta');
subplot(312)
plot(data_pitch(:,1),data_pitch(:,2),data_pitch(:,1),data_pitch(:,3),'linewidth',2);
grid;
title('Respuesta a SP onda cuadrada');
xlabel('Tiempo (seg)');
ylabel('Cabeceo (grados)');
subplot(313)
plot(data_roll(:,1),data_roll(:,2),data_roll(:,1),data_roll(:,3),'linewidth',2); grid;
title('Respuesta a SP onda cuadrada');
xlabel('Tiempo (seg)');
ylabel('Balance (grados)');
% Imprimir datos de tension aplicada a motores
figure(2)
subplot(221)
plot(data_Vf(:,1),data_Vf(:,2),'linewidth',2); grid;
title('Tension aplicada motor frontal');
xlabel('Tiempo (seg)');
ylabel('Vf (voltios)');
subplot(222)
plot(data_Vb(:,1),data_Vb(:,2),'linewidth',2); grid;
title('Tension aplicada motor posterior');
xlabel('Tiempo (seg)');
ylabel('Vb (voltios)');
subplot(223)
plot(data_Vr(:,1),data_Vr(:,2),'linewidth',2); grid;
title('Tension aplicada motor derecho');
xlabel('Tiempo (seg)');
ylabel('Vr (voltios)');
subplot(224)
plot(data_Vl(:,1),data_Vl(:,2),'linewidth',2); grid;
title('Tension aplicada motor izquierdo');
xlabel('Tiempo (seg)');
ylabel('Vl (voltios)');