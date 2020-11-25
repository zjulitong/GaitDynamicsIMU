function F_Joi = Fun_F_Joi(in1,in2,in3,in4,in5,in6,in7,in8,in9,in10,in11)
%FUN_F_JOI
%    F_JOI = FUN_F_JOI(IN1,IN2,IN3,IN4,IN5,IN6,IN7,IN8,IN9,IN10,IN11)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:45:03

Ls = in2(:,5);
Lsm = in2(:,6);
Lt = in2(:,3);
Ltm = in2(:,4);
cts_FLx1 = in10(:,1);
cts_FLx2 = in10(:,2);
cts_FLy1 = in11(:,1);
cts_FLx3 = in10(:,3);
cts_FLy2 = in11(:,2);
cts_FLx4 = in10(:,4);
cts_FLy3 = in11(:,3);
cts_FLx5 = in10(:,5);
cts_FLy4 = in11(:,4);
cts_FLy5 = in11(:,5);
cts_FRx1 = in8(:,1);
cts_FRx2 = in8(:,2);
cts_FRy1 = in9(:,1);
cts_FRx3 = in8(:,3);
cts_FRy2 = in9(:,2);
cts_FRx4 = in8(:,4);
cts_FRy3 = in9(:,3);
cts_FRx5 = in8(:,5);
cts_FRy4 = in9(:,4);
cts_FRy5 = in9(:,5);
dfmx = in2(:,11);
dfmy = in2(:,12);
g = in1(:,9);
mf = in1(:,4);
ms = in1(:,3);
mt = in1(:,2);
qb = in5(:,3);
qbdot = in6(:,3);
qbddot = in7(:,3);
qla = in5(:,9);
qladot = in6(:,9);
qladdot = in7(:,9);
qlh = in5(:,7);
qlhdot = in6(:,7);
qlhddot = in7(:,7);
qlk = in5(:,8);
qlkdot = in6(:,8);
qlkddot = in7(:,8);
qra = in5(:,6);
qradot = in6(:,6);
qraddot = in7(:,6);
qrh = in5(:,4);
qrhdot = in6(:,4);
qrhddot = in7(:,4);
qrk = in5(:,5);
qrkdot = in6(:,5);
qrkddot = in7(:,5);
xbddot = in7(:,1);
ybddot = in7(:,2);
t2 = qb+qra+qrh+qrk;
t3 = cos(t2);
t4 = dfmx.*t3;
t5 = sin(t2);
t6 = qb+qrh+qrk;
t7 = sin(t6);
t8 = Ls.*t7;
t10 = dfmy.*t5;
t9 = t4+t8-t10;
t11 = t4-t10;
t12 = qradot.*t11;
t13 = qb+qrh;
t14 = sin(t13);
t15 = Lt.*t14;
t16 = t4+t8-t10+t15;
t17 = qrkdot.*t9;
t18 = qbdot.*t16;
t19 = qrhdot.*t16;
t20 = t12+t17+t18+t19;
t21 = dfmy.*t3;
t22 = dfmx.*t5;
t23 = cos(t6);
t24 = cos(t13);
t26 = Ls.*t23;
t28 = Lt.*t24;
t25 = t21+t22-t26-t28;
t27 = t21+t22;
t29 = t21+t22-t26;
t30 = qrkdot.*t29;
t31 = qradot.*t27;
t32 = qbdot.*t25;
t33 = qrhdot.*t25;
t34 = t30+t31+t32+t33;
t35 = qb+qla+qlh+qlk;
t36 = cos(t35);
t37 = dfmx.*t36;
t38 = sin(t35);
t39 = qb+qlh+qlk;
t40 = sin(t39);
t41 = Ls.*t40;
t43 = dfmy.*t38;
t42 = t37+t41-t43;
t44 = t37-t43;
t45 = qladot.*t44;
t46 = qb+qlh;
t47 = sin(t46);
t48 = Lt.*t47;
t49 = t37+t41-t43+t48;
t50 = qlkdot.*t42;
t51 = qbdot.*t49;
t52 = qlhdot.*t49;
t53 = t45+t50+t51+t52;
t54 = dfmy.*t36;
t55 = dfmx.*t38;
t56 = cos(t39);
t57 = cos(t46);
t59 = Ls.*t56;
t61 = Lt.*t57;
t58 = t54+t55-t59-t61;
t60 = t54+t55;
t62 = t54+t55-t59;
t63 = qlkdot.*t62;
t64 = qladot.*t60;
t65 = qbdot.*t58;
t66 = qlhdot.*t58;
t67 = t63+t64+t65+t66;
t68 = qbdot.*t9;
t69 = qrhdot.*t9;
t70 = t12+t17+t68+t69;
t71 = qrkdot.*t70;
t72 = qbdot.*t11;
t73 = qrhdot.*t11;
t74 = qrkdot.*t11;
t75 = t12+t72+t73+t74;
t76 = qradot.*t75;
t77 = qbdot.*t20;
t78 = qrhdot.*t20;
t79 = qraddot.*t27;
t80 = qbddot.*t25;
t81 = qrhddot.*t25;
t82 = Lsm.*t7;
t83 = t15+t82;
t84 = qbdot.*t83;
t85 = qrhdot.*t83;
t86 = Lsm.*qrkdot.*t7;
t87 = t84+t85+t86;
t88 = Lsm.*t23;
t89 = t28+t88;
t90 = Lsm.*qrkdot.*t23;
t91 = qbdot.*t89;
t92 = qrhdot.*t89;
t93 = t90+t91+t92;
t94 = qbddot.*t16;
t95 = qrhddot.*t16;
t96 = qbdot.*t27;
t97 = qrhdot.*t27;
t98 = qrkdot.*t27;
t99 = t31+t96+t97+t98;
t100 = qrkddot.*t9;
t101 = qraddot.*t11;
t102 = qbdot.*t29;
t103 = qrhdot.*t29;
t104 = t30+t31+t102+t103;
t170 = qbdot.*t34;
t171 = qrhdot.*t34;
t172 = qradot.*t99;
t173 = qrkdot.*t104;
t105 = t94+t95+t100+t101-t170-t171-t172-t173+ybddot;
t106 = qbdot.*t42;
t107 = qlhdot.*t42;
t108 = t45+t50+t106+t107;
t109 = qlkdot.*t108;
t110 = qbdot.*t44;
t111 = qlhdot.*t44;
t112 = qlkdot.*t44;
t113 = t45+t110+t111+t112;
t114 = qladot.*t113;
t115 = qbdot.*t53;
t116 = qlhdot.*t53;
t117 = qladdot.*t60;
t118 = qbddot.*t58;
t119 = qlhddot.*t58;
t120 = Lsm.*t40;
t121 = t48+t120;
t122 = qbdot.*t121;
t123 = qlhdot.*t121;
t124 = Lsm.*qlkdot.*t40;
t125 = t122+t123+t124;
t126 = Lsm.*t56;
t127 = t61+t126;
t128 = Lsm.*qlkdot.*t56;
t129 = qbdot.*t127;
t130 = qlhdot.*t127;
t131 = t128+t129+t130;
t132 = qbddot.*t49;
t133 = qlhddot.*t49;
t134 = qbdot.*t60;
t135 = qlhdot.*t60;
t136 = qlkdot.*t60;
t137 = t64+t134+t135+t136;
t138 = qlkddot.*t42;
t139 = qladdot.*t44;
t140 = qbdot.*t62;
t141 = qlhdot.*t62;
t142 = t63+t64+t140+t141;
t197 = qbdot.*t67;
t198 = qlhdot.*t67;
t199 = qladot.*t137;
t200 = qlkdot.*t142;
t143 = t132+t133+t138+t139-t197-t198-t199-t200+ybddot;
t144 = qrkddot.*t29;
t145 = t71+t76+t77+t78+t79+t80+t81+t144-xbddot;
t146 = mf.*t145;
t147 = Ltm.*qbdot.*t14;
t148 = Ltm.*qrhdot.*t14;
t149 = t147+t148;
t150 = qbddot.*t89;
t151 = qrhddot.*t89;
t152 = Lsm.*qbdot.*t7;
t153 = Lsm.*qrhdot.*t7;
t154 = t86+t152+t153;
t155 = Lsm.*qrkddot.*t23;
t156 = t150+t151+t155+xbddot-qbdot.*t87-qrhdot.*t87-qrkdot.*t154;
t157 = qbddot.*t83;
t158 = qrhddot.*t83;
t159 = Lsm.*qbdot.*t23;
t160 = Lsm.*qrhdot.*t23;
t161 = t90+t159+t160;
t162 = qrkdot.*t161;
t163 = qbdot.*t93;
t164 = qrhdot.*t93;
t165 = Lsm.*qrkddot.*t7;
t166 = t157+t158+t162+t163+t164+t165+ybddot;
t167 = Ltm.*qbdot.*t24;
t168 = Ltm.*qrhdot.*t24;
t169 = t167+t168;
t174 = qlkddot.*t62;
t175 = t109+t114+t115+t116+t117+t118+t119+t174-xbddot;
t176 = mf.*t175;
t177 = Ltm.*qbdot.*t47;
t178 = Ltm.*qlhdot.*t47;
t179 = t177+t178;
t180 = qbddot.*t127;
t181 = qlhddot.*t127;
t182 = Lsm.*qbdot.*t40;
t183 = Lsm.*qlhdot.*t40;
t184 = t124+t182+t183;
t185 = Lsm.*qlkddot.*t56;
t186 = t180+t181+t185+xbddot-qbdot.*t125-qlhdot.*t125-qlkdot.*t184;
t187 = qbddot.*t121;
t188 = qlhddot.*t121;
t189 = Lsm.*qbdot.*t56;
t190 = Lsm.*qlhdot.*t56;
t191 = t128+t189+t190;
t192 = qlkdot.*t191;
t193 = qbdot.*t131;
t194 = qlhdot.*t131;
t195 = Lsm.*qlkddot.*t40;
t196 = t187+t188+t192+t193+t194+t195+ybddot;
t201 = Ltm.*qbdot.*t57;
t202 = Ltm.*qlhdot.*t57;
t203 = t201+t202;
F_Joi = [cts_FRx1+cts_FRx2+cts_FRx3+cts_FRx4+cts_FRx5+mf.*(t71+t76+t77+t78+t79+t80+t81-xbddot+qrkddot.*(t21+t22-Ls.*t23)),cts_FRy1+cts_FRy2+cts_FRy3+cts_FRy4+cts_FRy5-g.*mf-mf.*t105,cts_FLx1+cts_FLx2+cts_FLx3+cts_FLx4+cts_FLx5+mf.*(t109+t114+t115+t116+t117+t118+t119-xbddot+qlkddot.*(t54+t55-Ls.*t56)),cts_FLy1+cts_FLy2+cts_FLy3+cts_FLy4+cts_FLy5-g.*mf-mf.*t143,cts_FRx1+cts_FRx2+cts_FRx3+cts_FRx4+cts_FRx5+t146-ms.*t156,cts_FRy1+cts_FRy2+cts_FRy3+cts_FRy4+cts_FRy5-g.*mf-g.*ms-mf.*t105-ms.*t166,cts_FLx1+cts_FLx2+cts_FLx3+cts_FLx4+cts_FLx5+t176-ms.*t186,cts_FLy1+cts_FLy2+cts_FLy3+cts_FLy4+cts_FLy5-g.*mf-g.*ms-mf.*t143-ms.*t196,cts_FRx1+cts_FRx2+cts_FRx3+cts_FRx4+cts_FRx5+t146-ms.*t156-mt.*(xbddot-qbdot.*t149-qrhdot.*t149+Ltm.*qbddot.*t24+Ltm.*qrhddot.*t24),cts_FRy1+cts_FRy2+cts_FRy3+cts_FRy4+cts_FRy5-g.*mf-g.*ms-g.*mt-mf.*t105-ms.*t166-mt.*(ybddot+qbdot.*t169+qrhdot.*t169+Ltm.*qbddot.*t14+Ltm.*qrhddot.*t14),cts_FLx1+cts_FLx2+cts_FLx3+cts_FLx4+cts_FLx5+t176-ms.*t186-mt.*(xbddot-qbdot.*t179-qlhdot.*t179+Ltm.*qbddot.*t57+Ltm.*qlhddot.*t57),cts_FLy1+cts_FLy2+cts_FLy3+cts_FLy4+cts_FLy5-g.*mf-g.*ms-g.*mt-mf.*t143-ms.*t196-mt.*(ybddot+qbdot.*t203+qlhdot.*t203+Ltm.*qbddot.*t47+Ltm.*qlhddot.*t47)];