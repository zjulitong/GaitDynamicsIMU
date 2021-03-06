function J_pJoi = Fun_J_posiJoi(in1,in2)
%FUN_J_POSIJOI
%    J_PJOI = FUN_J_POSIJOI(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:38

Ls = in2(:,5);
Lt = in2(:,3);
qb = in1(:,3);
qlh = in1(:,7);
qlk = in1(:,8);
qrh = in1(:,4);
qrk = in1(:,5);
t2 = qb+qrh;
t3 = cos(t2);
t4 = Lt.*t3;
t5 = sin(t2);
t6 = Lt.*t5;
t7 = qb+qrh+qrk;
t8 = cos(t7);
t9 = Ls.*t8;
t10 = t4+t9;
t11 = sin(t7);
t12 = Ls.*t11;
t13 = t6+t12;
t14 = qb+qlh;
t15 = cos(t14);
t16 = Lt.*t15;
t17 = sin(t14);
t18 = Lt.*t17;
t19 = qb+qlh+qlk;
t20 = cos(t19);
t21 = Ls.*t20;
t22 = t16+t21;
t23 = sin(t19);
t24 = Ls.*t23;
t25 = t18+t24;
J_pJoi = reshape([1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,0.0,t4,t6,t10,t13,0.0,0.0,t16,t18,t22,t25,0.0,0.0,t4,t6,t10,t13,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t9,t12,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t16,t18,t22,t25,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t21,t24,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[12,9]);
