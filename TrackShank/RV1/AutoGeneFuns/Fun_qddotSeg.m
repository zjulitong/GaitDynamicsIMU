function qddotSeg = Fun_qddotSeg(in1)
%FUN_QDDOTSEG
%    QDDOTSEG = FUN_QDDOTSEG(IN1)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:32

qbddot = in1(:,3);
qladdot = in1(:,9);
qlhddot = in1(:,7);
qlkddot = in1(:,8);
qraddot = in1(:,6);
qrhddot = in1(:,4);
qrkddot = in1(:,5);
qddotSeg = [qbddot+qrhddot;qbddot+qrhddot+qrkddot;qbddot+qraddot+qrhddot+qrkddot;qbddot+qlhddot;qbddot+qlhddot+qlkddot;qbddot+qladdot+qlhddot+qlkddot];
