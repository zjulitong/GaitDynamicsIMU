function yn = Fun_yn(in1,in2)
%FUN_YN
%    YN = FUN_YN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:36

Lb = in2(:,1);
qb = in1(:,3);
yb = in1(:,2);
yn = yb+Lb.*cos(qb);
