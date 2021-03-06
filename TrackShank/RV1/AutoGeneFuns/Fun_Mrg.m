function Mrg = Fun_Mrg(in1,in2,in3,in4,in5,in6)
%FUN_MRG
%    MRG = FUN_MRG(IN1,IN2,IN3,IN4,IN5,IN6)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:55

Ls = in6(:,5);
Lt = in6(:,3);
ctsx1 = in3(:,1);
ctsx2 = in3(:,2);
ctsy1 = in4(:,1);
ctsx3 = in3(:,3);
ctsy2 = in4(:,2);
ctsx4 = in3(:,4);
ctsy3 = in4(:,3);
ctsx5 = in3(:,5);
ctsy4 = in4(:,4);
ctsy5 = in4(:,5);
cts_FRx1 = in1(:,1);
cts_FRx2 = in1(:,2);
cts_FRy1 = in2(:,1);
cts_FRx3 = in1(:,3);
cts_FRy2 = in2(:,2);
cts_FRx4 = in1(:,4);
cts_FRy3 = in2(:,3);
cts_FRx5 = in1(:,5);
cts_FRy4 = in2(:,4);
cts_FRy5 = in2(:,5);
dfmx = in6(:,11);
dfmy = in6(:,12);
qb = in5(:,3);
qra = in5(:,6);
qrh = in5(:,4);
qrk = in5(:,5);
yb = in5(:,2);
t2 = qb+qra+qrh+qrk;
t3 = cos(t2);
t4 = dfmy.*t3;
t5 = qb+qrh;
t6 = cos(t5);
t7 = sin(t2);
t8 = dfmx.*t7;
t9 = qb+qrh+qrk;
t10 = cos(t9);
t12 = Lt.*t6;
t13 = Ls.*t10;
t11 = t4+t8-t12-t13+yb;
t14 = dfmy.*t7;
Mrg = cts_FRx1.*t11+cts_FRx2.*t11+cts_FRx3.*t11+cts_FRx4.*t11+cts_FRx5.*t11+cts_FRy1.*(t14+ctsx1.*t3-ctsy1.*t7-dfmx.*t3)+cts_FRy2.*(t14+ctsx2.*t3-ctsy2.*t7-dfmx.*t3)+cts_FRy3.*(t14+ctsx3.*t3-ctsy3.*t7-dfmx.*t3)+cts_FRy4.*(t14+ctsx4.*t3-ctsy4.*t7-dfmx.*t3)+cts_FRy5.*(t14+ctsx5.*t3-ctsy5.*t7-dfmx.*t3);
