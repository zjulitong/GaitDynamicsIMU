function qrsdot = Fun_qrsdot(in1)
%FUN_QRSDOT
%    QRSDOT = FUN_QRSDOT(IN1)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:31

qbdot = in1(:,3);
qrhdot = in1(:,4);
qrkdot = in1(:,5);
qrsdot = qbdot+qrhdot+qrkdot;