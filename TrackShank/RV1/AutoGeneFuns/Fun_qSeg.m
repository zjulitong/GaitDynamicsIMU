function qSeg = Fun_qSeg(in1)
%FUN_QSEG
%    QSEG = FUN_QSEG(IN1)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:31

qb = in1(:,3);
qla = in1(:,9);
qlh = in1(:,7);
qlk = in1(:,8);
qra = in1(:,6);
qrh = in1(:,4);
qrk = in1(:,5);
qSeg = [qb+qrh;qb+qrh+qrk;qb+qra+qrh+qrk;qb+qlh;qb+qlh+qlk;qb+qla+qlh+qlk];
