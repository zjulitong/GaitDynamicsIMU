function qltdot = Fun_qltdot(in1)
%FUN_QLTDOT
%    QLTDOT = FUN_QLTDOT(IN1)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:31

qbdot = in1(:,3);
qlhdot = in1(:,7);
qltdot = qbdot+qlhdot;
