function Fy_tot = Fun_Fy_tot(in1,in2,in3,in4,in5)
%FUN_FY_TOT
%    FY_TOT = FUN_FY_TOT(IN1,IN2,IN3,IN4,IN5)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:48

Lbm = in5(:,2);
Ls = in5(:,5);
Lsm = in5(:,6);
Lt = in5(:,3);
Ltm = in5(:,4);
dfmx = in5(:,11);
dfmy = in5(:,12);
g = in4(:,9);
mb = in4(:,1);
mf = in4(:,4);
ms = in4(:,3);
mt = in4(:,2);
qb = in1(:,3);
qbdot = in2(:,3);
qbddot = in3(:,3);
qla = in1(:,9);
qladot = in2(:,9);
qladdot = in3(:,9);
qlh = in1(:,7);
qlhdot = in2(:,7);
qlhddot = in3(:,7);
qlk = in1(:,8);
qlkdot = in2(:,8);
qlkddot = in3(:,8);
qra = in1(:,6);
qradot = in2(:,6);
qraddot = in3(:,6);
qrh = in1(:,4);
qrhdot = in2(:,4);
qrhddot = in3(:,4);
qrk = in1(:,5);
qrkdot = in2(:,5);
qrkddot = in3(:,5);
ybddot = in3(:,2);
t2 = qb+qla+qlh+qlk;
t3 = cos(t2);
t4 = dfmx.*t3;
t5 = sin(t2);
t6 = qb+qlh;
t7 = sin(t6);
t8 = Lt.*t7;
t9 = qb+qlh+qlk;
t10 = sin(t9);
t11 = Ls.*t10;
t27 = dfmy.*t5;
t12 = t4+t8+t11-t27;
t13 = dfmy.*t3;
t14 = dfmx.*t5;
t15 = cos(t9);
t16 = cos(t6);
t18 = Ls.*t15;
t23 = Lt.*t16;
t17 = t13+t14-t18-t23;
t19 = t13+t14-t18;
t20 = qlkdot.*t19;
t21 = t13+t14;
t22 = qladot.*t21;
t24 = qbdot.*t17;
t25 = qlhdot.*t17;
t26 = t20+t22+t24+t25;
t28 = Ltm.*qbdot.*t16;
t29 = Ltm.*qlhdot.*t16;
t30 = t28+t29;
t31 = qb+qrh;
t32 = cos(t31);
t33 = Ltm.*qbdot.*t32;
t34 = Ltm.*qrhdot.*t32;
t35 = t33+t34;
t36 = sin(t31);
t37 = qb+qra+qrh+qrk;
t38 = cos(t37);
t39 = dfmx.*t38;
t40 = sin(t37);
t41 = Lt.*t36;
t42 = qb+qrh+qrk;
t43 = sin(t42);
t44 = Ls.*t43;
t59 = dfmy.*t40;
t45 = t39+t41+t44-t59;
t46 = dfmy.*t38;
t47 = dfmx.*t40;
t48 = cos(t42);
t50 = Ls.*t48;
t55 = Lt.*t32;
t49 = t46+t47-t50-t55;
t51 = t46+t47-t50;
t52 = qrkdot.*t51;
t53 = t46+t47;
t54 = qradot.*t53;
t56 = qbdot.*t49;
t57 = qrhdot.*t49;
t58 = t52+t54+t56+t57;
t60 = Lsm.*t10;
t61 = t8+t60;
t62 = Lsm.*t15;
t63 = t23+t62;
t64 = Lsm.*qlkdot.*t15;
t65 = qbdot.*t63;
t66 = qlhdot.*t63;
t67 = t64+t65+t66;
t68 = Lsm.*t43;
t69 = t41+t68;
t70 = Lsm.*t48;
t71 = t55+t70;
t72 = Lsm.*qrkdot.*t48;
t73 = qbdot.*t71;
t74 = qrhdot.*t71;
t75 = t72+t73+t74;
Fy_tot = mb.*(g+ybddot-Lbm.*qbddot.*sin(qb)-Lbm.*qbdot.^2.*cos(qb))+ms.*(g+ybddot+qbdot.*t67+qbddot.*t61+qlhdot.*t67+qlhddot.*t61+qlkdot.*(t64+Lsm.*qbdot.*t15+Lsm.*qlhdot.*t15)+Lsm.*qlkddot.*t10)+ms.*(g+ybddot+qbdot.*t75+qbddot.*t69+qrhdot.*t75+qrhddot.*t69+qrkdot.*(t72+Lsm.*qbdot.*t48+Lsm.*qrhdot.*t48)+Lsm.*qrkddot.*t43)+mf.*(g+ybddot-qbdot.*t26+qbddot.*t12-qlhdot.*t26+qlhddot.*t12-qladot.*(t22+qbdot.*t21+qlhdot.*t21+qlkdot.*t21)+qladdot.*(t4-t27)-qlkdot.*(t20+t22+qbdot.*t19+qlhdot.*t19)+qlkddot.*(t4+t11-t27))+mf.*(g+ybddot-qbdot.*t58+qbddot.*t45-qrhdot.*t58+qrhddot.*t45-qradot.*(t54+qbdot.*t53+qrhdot.*t53+qrkdot.*t53)+qraddot.*(t39-t59)-qrkdot.*(t52+t54+qbdot.*t51+qrhdot.*t51)+qrkddot.*(t39+t44-t59))+mt.*(g+ybddot+qbdot.*t30+qlhdot.*t30+Ltm.*qbddot.*t7+Ltm.*qlhddot.*t7)+mt.*(g+ybddot+qbdot.*t35+qrhdot.*t35+Ltm.*qbddot.*t36+Ltm.*qrhddot.*t36);