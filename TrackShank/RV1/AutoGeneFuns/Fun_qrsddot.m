function qrsddot = Fun_qrsddot(in1)
%FUN_QRSDDOT
%    QRSDDOT = FUN_QRSDDOT(IN1)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:31

qbddot = in1(:,3);
qrhddot = in1(:,4);
qrkddot = in1(:,5);
qrsddot = qbddot+qrhddot+qrkddot;