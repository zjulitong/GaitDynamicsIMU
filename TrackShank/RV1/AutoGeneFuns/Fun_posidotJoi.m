function pdotJoi = Fun_posidotJoi(in1,in2,in3)
%FUN_POSIDOTJOI
%    PDOTJOI = FUN_POSIDOTJOI(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:37

Ls = in3(:,5);
Lt = in3(:,3);
qb = in1(:,3);
qbdot = in2(:,3);
qlh = in1(:,7);
qlhdot = in2(:,7);
qlk = in1(:,8);
qlkdot = in2(:,8);
qrh = in1(:,4);
qrhdot = in2(:,4);
qrk = in1(:,5);
qrkdot = in2(:,5);
xbdot = in2(:,1);
ybdot = in2(:,2);
t2 = qb+qrh;
t3 = cos(t2);
t4 = sin(t2);
t5 = Lt.*t3;
t6 = qb+qrh+qrk;
t7 = cos(t6);
t8 = Ls.*t7;
t9 = t5+t8;
t10 = Lt.*t4;
t11 = sin(t6);
t12 = Ls.*t11;
t13 = t10+t12;
t14 = qb+qlh;
t15 = cos(t14);
t16 = sin(t14);
t17 = Lt.*t15;
t18 = qb+qlh+qlk;
t19 = cos(t18);
t20 = Ls.*t19;
t21 = t17+t20;
t22 = Lt.*t16;
t23 = sin(t18);
t24 = Ls.*t23;
t25 = t22+t24;
pdotJoi = [xbdot,ybdot,xbdot+Lt.*qbdot.*t3+Lt.*qrhdot.*t3,ybdot+Lt.*qbdot.*t4+Lt.*qrhdot.*t4,xbdot+qbdot.*t9+qrhdot.*t9+Ls.*qrkdot.*t7,ybdot+qbdot.*t13+qrhdot.*t13+Ls.*qrkdot.*t11,xbdot,ybdot,xbdot+Lt.*qbdot.*t15+Lt.*qlhdot.*t15,ybdot+Lt.*qbdot.*t16+Lt.*qlhdot.*t16,xbdot+qbdot.*t21+qlhdot.*t21+Ls.*qlkdot.*t19,ybdot+qbdot.*t25+qlhdot.*t25+Ls.*qlkdot.*t23];
