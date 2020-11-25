function yrsm = Fun_yrsm(in1,in2)
%FUN_YRSM
%    YRSM = FUN_YRSM(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:34

Lsm = in2(:,6);
Lt = in2(:,3);
qb = in1(:,3);
qrh = in1(:,4);
qrk = in1(:,5);
yb = in1(:,2);
yrsm = yb-Lt.*cos(qb+qrh)-Lsm.*cos(qb+qrh+qrk);