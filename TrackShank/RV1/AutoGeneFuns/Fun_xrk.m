function xrk = Fun_xrk(in1,in2)
%FUN_XRK
%    XRK = FUN_XRK(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:32

Lt = in2(:,3);
qb = in1(:,3);
qrh = in1(:,4);
xb = in1(:,1);
xrk = xb+Lt.*sin(qb+qrh);
