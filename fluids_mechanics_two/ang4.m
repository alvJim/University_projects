function f = ang4(t,y1,c)

f(1,1) = y1(2,1);
f(2,1) = -(9.81/0.5)*sin(y1(1,1)) - c*y1(2,1);

end