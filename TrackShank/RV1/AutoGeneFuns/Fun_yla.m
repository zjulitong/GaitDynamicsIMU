function yla = Fun_yla(in1,in2)
%FUN_YLA
%    YLA = FUN_YLA(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:33

Ls = in2(:,5);
Lt = in2(:,3);
qb = in1(:,3);
qlh = in1(:,7);
qlk = in1(:,8);
yb = in1(:,2);
yla = yb-Lt.*cos(qb+qlh)-Ls.*cos(qb+qlh+qlk);