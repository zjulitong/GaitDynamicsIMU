function J_ylctsdot = Fun_J_ylctsdot(in1,in2,in3,in4,in5)
%FUN_J_YLCTSDOT
%    J_YLCTSDOT = FUN_J_YLCTSDOT(IN1,IN2,IN3,IN4,IN5)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:54

Ls = in3(:,5);
Lt = in3(:,3);
ctsx1 = in4(:,1);
ctsx2 = in4(:,2);
ctsy1 = in5(:,1);
ctsx3 = in4(:,3);
ctsy2 = in5(:,2);
ctsx4 = in4(:,4);
ctsy3 = in5(:,3);
ctsx5 = in4(:,5);
ctsy4 = in5(:,4);
ctsy5 = in5(:,5);
qb = in1(:,3);
qbdot = in2(:,3);
qla = in1(:,9);
qladot = in2(:,9);
qlh = in1(:,7);
qlhdot = in2(:,7);
qlk = in1(:,8);
qlkdot = in2(:,8);
t2 = qb+qla+qlh+qlk;
t3 = cos(t2);
t4 = ctsy1.*t3;
t5 = sin(t2);
t6 = ctsx1.*t5;
t7 = qb+qlh+qlk;
t8 = cos(t7);
t9 = qb+qlh;
t10 = cos(t9);
t12 = Ls.*t8;
t14 = Lt.*t10;
t11 = t4+t6-t12-t14;
t13 = t4+t6;
t15 = t4+t6-t12;
t16 = ctsx1.*t3;
t17 = sin(t9);
t18 = Lt.*t17;
t19 = sin(t7);
t20 = Ls.*t19;
t22 = ctsy1.*t5;
t21 = t16+t18+t20-t22;
t23 = ctsy2.*t3;
t24 = ctsx2.*t5;
t25 = -t12+t23+t24;
t26 = t23+t24;
t27 = qbdot.*(t12+t14-t23-t24);
t28 = qlhdot.*(t12+t14-t23-t24);
t30 = qlkdot.*t25;
t31 = qladot.*t26;
t29 = t27+t28-t30-t31;
t32 = ctsx2.*t3;
t34 = ctsy2.*t5;
t33 = t18+t20+t32-t34;
t35 = ctsy3.*t3;
t36 = ctsx3.*t5;
t37 = -t12+t35+t36;
t38 = t35+t36;
t39 = qbdot.*(t12+t14-t35-t36);
t40 = qlhdot.*(t12+t14-t35-t36);
t42 = qlkdot.*t37;
t43 = qladot.*t38;
t41 = t39+t40-t42-t43;
t44 = ctsx3.*t3;
t46 = ctsy3.*t5;
t45 = t18+t20+t44-t46;
t47 = ctsy4.*t3;
t48 = ctsx4.*t5;
t49 = -t12+t47+t48;
t50 = t47+t48;
t51 = qbdot.*(t12+t14-t47-t48);
t52 = qlhdot.*(t12+t14-t47-t48);
t54 = qlkdot.*t49;
t55 = qladot.*t50;
t53 = t51+t52-t54-t55;
t56 = ctsx4.*t3;
t58 = ctsy4.*t5;
t57 = t18+t20+t56-t58;
t59 = ctsy5.*t3;
t60 = ctsx5.*t5;
t61 = -t12+t59+t60;
t62 = t59+t60;
t63 = qbdot.*(t12+t14-t59-t60);
t64 = qlhdot.*(t12+t14-t59-t60);
t66 = qlkdot.*t61;
t67 = qladot.*t62;
t65 = t63+t64-t66-t67;
t68 = ctsx5.*t3;
t70 = ctsy5.*t5;
t69 = t18+t20+t68-t70;
J_ylctsdot = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-qlkdot.*(t4+t6-Ls.*t8)-qbdot.*t11-qladot.*t13-qlhdot.*t11,t29,t41,t53,t65,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-qbdot.*t11-qladot.*t13-qlhdot.*t11-qlkdot.*t15,t29,t41,t53,t65,-qbdot.*t15-qladot.*t13-qlhdot.*t15-qlkdot.*t15,-t30-t31-qbdot.*t25-qlhdot.*t25,-t42-t43-qbdot.*t37-qlhdot.*t37,-t54-t55-qbdot.*t49-qlhdot.*t49,-t66-t67-qbdot.*t61-qlhdot.*t61,-qbdot.*t13-qladot.*t13-qlhdot.*t13-qlkdot.*t13,-t31-qbdot.*t26-qlhdot.*t26-qlkdot.*t26,-t43-qbdot.*t38-qlhdot.*t38-qlkdot.*t38,-t55-qbdot.*t50-qlhdot.*t50-qlkdot.*t50,-t67-qbdot.*t62-qlhdot.*t62-qlkdot.*t62,0.0,0.0,0.0,0.0,0.0,1.0,1.0,1.0,1.0,1.0,t21,t33,t45,t57,t69,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t21,t33,t45,t57,t69,t16+t20-t22,t20+t32-t34,t20+t44-t46,t20+t56-t58,t20+t68-t70,t16-t22,t32-t34,t44-t46,t56-t58,t68-t70],[5,18]);
