function J_xrcts = Fun_J_xrcts(in1,in2,in3,in4)
%FUN_J_XRCTS
%    J_XRCTS = FUN_J_XRCTS(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:48

Ls = in2(:,5);
Lt = in2(:,3);
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
qb = in1(:,3);
qra = in1(:,6);
qrh = in1(:,4);
qrk = in1(:,5);
t2 = qb+qra+qrh+qrk;
t3 = cos(t2);
t4 = qb+qrh;
t5 = cos(t4);
t6 = Lt.*t5;
t7 = sin(t2);
t8 = qb+qrh+qrk;
t9 = cos(t8);
t10 = Ls.*t9;
t12 = ctsy1.*t3;
t13 = ctsx1.*t7;
t11 = t6+t10-t12-t13;
t15 = ctsy2.*t3;
t16 = ctsx2.*t7;
t14 = t6+t10-t15-t16;
t18 = ctsy3.*t3;
t19 = ctsx3.*t7;
t17 = t6+t10-t18-t19;
t21 = ctsy4.*t3;
t22 = ctsx4.*t7;
t20 = t6+t10-t21-t22;
t24 = ctsy5.*t3;
t25 = ctsx5.*t7;
t23 = t6+t10-t24-t25;
J_xrcts = reshape([1.0,1.0,1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,t11,t14,t17,t20,t23,t11,t14,t17,t20,t23,t10-t12-t13,t10-t15-t16,t10-t18-t19,t10-t21-t22,t10-t24-t25,-t12-t13,-t15-t16,-t18-t19,-t21-t22,-t24-t25,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[5,9]);
