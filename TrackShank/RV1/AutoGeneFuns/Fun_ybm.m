function ybm = Fun_ybm(in1,in2)
%FUN_YBM
%    YBM = FUN_YBM(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:36

Lbm = in2(:,2);
qb = in1(:,3);
yb = in1(:,2);
ybm = yb+Lbm.*cos(qb);