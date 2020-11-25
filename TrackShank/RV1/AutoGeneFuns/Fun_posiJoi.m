function pJoi = Fun_posiJoi(in1,in2)
%FUN_POSIJOI
%    PJOI = FUN_POSIJOI(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:33

Ls = in2(:,5);
Lt = in2(:,3);
qb = in1(:,3);
qlh = in1(:,7);
qlk = in1(:,8);
qrh = in1(:,4);
qrk = in1(:,5);
xb = in1(:,1);
yb = in1(:,2);
t2 = qb+qrh;
t3 = sin(t2);
t4 = Lt.*t3;
t5 = cos(t2);
t6 = qb+qrh+qrk;
t7 = qb+qlh;
t8 = sin(t7);
t9 = Lt.*t8;
t10 = cos(t7);
t11 = qb+qlh+qlk;
pJoi = [xb,yb,t4+xb,yb-Lt.*t5,t4+xb+Ls.*sin(t6),yb-Lt.*t5-Ls.*cos(t6),xb,yb,t9+xb,yb-Lt.*t10,t9+xb+Ls.*sin(t11),yb-Lt.*t10-Ls.*cos(t11)];
