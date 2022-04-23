% Matlab code

controltotal = 0;
preci = 1 ;
while controltotal == 0

controlV = 0;

V1 = 1 ;
V = V1 ;
u_ini = V*cos(alfa);
w_ini = V*sin(alfa);
sim('Trabajo', 0);
Fz1 = Fuerzas(3);

V2 = 340 ;
V = V2 ;
u_ini = V*cos(alfa);
w_ini = V*sin(alfa);
sim('Trabajo', 0);
Fz2 = Fuerzas(3);

while controlV == 0
  peso1 = 1-abs(Fz1)/abs(Fz2-Fz1);
  peso2 = 1-abs(Fz2)/abs(Fz2-Fz1);
  V_med = sqrt(V1*V1*peso1 + V2*V2*peso2) ;
  V = V_med;
  u_ini = V*cos(alfa);
  w_ini = V*sin(alfa);
  sim('Trabajo', 0);
  Fz_med = Fuerzas(3);

  if abs(Fz_med) < preci
    controlV = 1;
  elseif Fz_med*Fz1<0
    V2 = V_med;
    Fz2 = Fz_med;
  elseif Fz_med*Fz2<0
    V1 = V_med;
    Fz1 = Fz_med;
  end
end
V = V_med ;

controlpospalanca=0;

pospalanca1 = 0 ;
pospalanca = pospalanca1 ;
sim('Trabajo', 0);
Fx1 = Fuerzas(1);

pospalanca2 = 1 ;
pospalanca = pospalanca2 ;
sim('Trabajo', 0);
Fx2 = Fuerzas(1);

while controlpospalanca == 0
  peso1 = 1-abs(Fx1)/abs(Fx2-Fx1);
  peso2 = 1-abs(Fx2)/abs(Fx2-Fx1);
  pospalanca_med = (pospalanca1*peso1 + pospalanca2*peso2) ;
  pospalanca = pospalanca_med;
  sim('Trabajo', 0);
  Fx_med = Fuerzas(1);

  if abs(Fx_med) < preci
    controlpospalanca = 1;
  elseif Fx_med*Fx1<0
    pospalanca2 = pospalanca_med;
    Fx2 = Fx_med;
  elseif Fx_med*Fx2<0
    pospalanca1 = pospalanca_med;
    Fx1 = Fx_med;
  end
end
pospalanca = pospalanca_med ;

controldelta=0;

delta1 = -30*pi/180 ;
delta = delta1 ;
sim('Trabajo', 0);
My1 = Fuerzas(2);

delta2 =  30*pi/180 ;
delta = delta2 ;
sim('Trabajo', 0);
My2 = Fuerzas(2);

while controldelta == 0
  peso1 = 1-abs(My1)/abs(My2-My1);
  peso2 = 1-abs(My2)/abs(My2-My1);
  delta_med = (delta1*peso1 + delta2*peso2) ;
  delta = delta_med;
  sim('Trabajo', 0);
  My_med = Fuerzas(2);

  if abs(My_med) < preci
    controldelta = 1;
  elseif(My_med*My1<0)
    delta2 = delta_med;
    My2 = My_med;
  elseif(My_med*My2<0)
    delta1 = delta_med;
    My1 = My_med;
  end
end
delta = delta_med ;

Fuerzas ;

if (abs(Fuerzas(1)) < preci) & (abs(Fuerzas(2)) < preci) & (abs(Fuerzas(3)) < preci)
  controltotal = 1;
end
end % while controltotal


%Descomentar para calcular puntos de equilibrio

V
pospalanca
delta
%deltagrados = delta*180/pi;
