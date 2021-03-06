function xrhe = Fun_xrhe(in1,in2)
%FUN_XRHE
%    XRHE = FUN_XRHE(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:34

Ls = in2(:,5);
Lt = in2(:,3);
dhex = in2(:,7);
dhey = in2(:,8);
qb = in1(:,3);
qra = in1(:,6);
qrh = in1(:,4);
qrk = in1(:,5);
xb = in1(:,1);
t2 = qb+qra+qrh+qrk;
xrhe = xb+Lt.*sin(qb+qrh)+dhex.*cos(t2)-dhey.*sin(t2)+Ls.*sin(qb+qrh+qrk);
