function T_Joi = Fun_T_Joi(in1,in2)
%FUN_T_JOI
%    T_JOI = FUN_T_JOI(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:44:14

Tlamax = in2(:,6);
Tlhmax = in2(:,4);
Tlkmax = in2(:,5);
Tramax = in2(:,3);
Trhmax = in2(:,1);
Trkmax = in2(:,2);
ala = in1(:,6);
alh = in1(:,4);
alk = in1(:,5);
ara = in1(:,3);
arh = in1(:,1);
ark = in1(:,2);
T_Joi = [Trhmax.*arh,Trkmax.*ark,Tramax.*ara,Tlhmax.*alh,Tlkmax.*alk,Tlamax.*ala];
