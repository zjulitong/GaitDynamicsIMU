function xrcts = Fun_xrcts(in1,in2,in3,in4)
%FUN_XRCTS
%    XRCTS = FUN_XRCTS(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:36

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
xb = in1(:,1);
t2 = qb+qra+qrh+qrk;
t3 = cos(t2);
t4 = sin(t2);
t5 = qb+qrh;
t6 = sin(t5);
t7 = Lt.*t6;
t8 = qb+qrh+qrk;
t9 = sin(t8);
t10 = Ls.*t9;
xrcts = [t7+t10+xb+ctsx1.*t3-ctsy1.*t4,t7+t10+xb+ctsx2.*t3-ctsy2.*t4,t7+t10+xb+ctsx3.*t3-ctsy3.*t4,t7+t10+xb+ctsx4.*t3-ctsy4.*t4,t7+t10+xb+ctsx5.*t3-ctsy5.*t4];