function xrfmddot = Fun_xrfmddot(in1,in2,in3,in4)
%FUN_XRFMDDOT
%    XRFMDDOT = FUN_XRFMDDOT(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:44

Ls = in4(:,5);
Lt = in4(:,3);
dfmx = in4(:,11);
dfmy = in4(:,12);
qb = in1(:,3);
qbdot = in2(:,3);
qbddot = in3(:,3);
qra = in1(:,6);
qradot = in2(:,6);
qraddot = in3(:,6);
qrh = in1(:,4);
qrhdot = in2(:,4);
qrhddot = in3(:,4);
qrk = in1(:,5);
qrkdot = in2(:,5);
qrkddot = in3(:,5);
xbddot = in3(:,1);
t2 = qb+qra+qrh+qrk;
t3 = cos(t2);
t4 = dfmx.*t3;
t5 = sin(t2);
t6 = qb+qrh+qrk;
t7 = sin(t6);
t8 = Ls.*t7;
t10 = dfmy.*t5;
t9 = t4+t8-t10;
t11 = t4-t10;
t12 = qradot.*t11;
t13 = qb+qrh;
t14 = sin(t13);
t15 = Lt.*t14;
t16 = t4+t8-t10+t15;
t17 = qrkdot.*t9;
t18 = qbdot.*t16;
t19 = qrhdot.*t16;
t20 = t12+t17+t18+t19;
t21 = dfmy.*t3;
t22 = dfmx.*t5;
t23 = cos(t6);
t24 = cos(t13);
t25 = t21+t22-Ls.*t23-Lt.*t24;
xrfmddot = xbddot-qraddot.*(t21+t22)-qrkddot.*(t21+t22-Ls.*t23)-qbdot.*t20-qbddot.*t25-qrhdot.*t20-qrhddot.*t25-qradot.*(t12+qbdot.*t11+qrhdot.*t11+qrkdot.*t11)-qrkdot.*(t12+t17+qbdot.*t9+qrhdot.*t9);