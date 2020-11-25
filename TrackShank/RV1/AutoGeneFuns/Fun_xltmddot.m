function xltmddot = Fun_xltmddot(in1,in2,in3,in4)
%FUN_XLTMDDOT
%    XLTMDDOT = FUN_XLTMDDOT(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:44

Ltm = in4(:,4);
qb = in1(:,3);
qbdot = in2(:,3);
qbddot = in3(:,3);
qlh = in1(:,7);
qlhdot = in2(:,7);
qlhddot = in3(:,7);
xbddot = in3(:,1);
t2 = qb+qlh;
t3 = sin(t2);
t4 = Ltm.*qbdot.*t3;
t5 = Ltm.*qlhdot.*t3;
t6 = t4+t5;
t7 = cos(t2);
xltmddot = xbddot-qbdot.*t6-qlhdot.*t6+Ltm.*qbddot.*t7+Ltm.*qlhddot.*t7;