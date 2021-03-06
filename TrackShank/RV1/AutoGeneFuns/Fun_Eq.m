function Eq = Fun_Eq(in1,in2,in3,in4,in5,in6,in7,in8,in9,in10,in11,in12)
%FUN_EQ
%    EQ = FUN_EQ(IN1,IN2,IN3,IN4,IN5,IN6,IN7,IN8,IN9,IN10,IN11,IN12)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    18-Nov-2020 15:45:23

Ib = in1(:,5);
If = in1(:,8);
Is = in1(:,7);
It = in1(:,6);
Lbm = in2(:,2);
Ls = in2(:,5);
Lsm = in2(:,6);
Lt = in2(:,3);
Ltm = in2(:,4);
Tla = in8(:,6);
Tlh = in8(:,4);
Tlk = in8(:,5);
Tra = in8(:,3);
Trh = in8(:,1);
Trk = in8(:,2);
ctsx1 = in3(:,1);
ctsx2 = in3(:,2);
ctsy1 = in4(:,1);
ctsx3 = in3(:,3);
ctsy2 = in4(:,2);
ctsx4 = in3(:,4);
ctsy3 = in4(:,3);
ctsx5 = in3(:,5);
ctsy4 = in4(:,4);
ctsy5 = in4(:,5);
cts_FLx1 = in11(:,1);
cts_FLx2 = in11(:,2);
cts_FLy1 = in12(:,1);
cts_FLx3 = in11(:,3);
cts_FLy2 = in12(:,2);
cts_FLx4 = in11(:,4);
cts_FLy3 = in12(:,3);
cts_FLx5 = in11(:,5);
cts_FLy4 = in12(:,4);
cts_FLy5 = in12(:,5);
cts_FRx1 = in9(:,1);
cts_FRx2 = in9(:,2);
cts_FRy1 = in10(:,1);
cts_FRx3 = in9(:,3);
cts_FRy2 = in10(:,2);
cts_FRx4 = in9(:,4);
cts_FRy3 = in10(:,3);
cts_FRx5 = in9(:,5);
cts_FRy4 = in10(:,4);
cts_FRy5 = in10(:,5);
dfmx = in2(:,11);
dfmy = in2(:,12);
g = in1(:,9);
mb = in1(:,1);
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
yb = in5(:,2);
ybddot = in7(:,2);
t2 = qb+qla+qlh+qlk;
t3 = cos(t2);
t4 = dfmx.*t3;
t5 = sin(t2);
t6 = qb+qlh+qlk;
t7 = sin(t6);
t8 = Ls.*t7;
t10 = dfmy.*t5;
t9 = t4+t8-t10;
t11 = t4-t10;
t12 = qladot.*t11;
t13 = qb+qlh;
t14 = sin(t13);
t15 = Lt.*t14;
t16 = t4+t8-t10+t15;
t17 = qlkdot.*t9;
t18 = qbdot.*t16;
t19 = qlhdot.*t16;
t20 = t12+t17+t18+t19;
t21 = dfmy.*t3;
t22 = dfmx.*t5;
t23 = cos(t6);
t24 = cos(t13);
t62 = Lt.*t24;
t85 = Ls.*t23;
t25 = t21+t22-t62-t85;
t26 = qb+qra+qrh+qrk;
t27 = cos(t26);
t28 = dfmx.*t27;
t29 = sin(t26);
t30 = qb+qrh+qrk;
t31 = sin(t30);
t32 = Ls.*t31;
t34 = dfmy.*t29;
t33 = t28+t32-t34;
t35 = t28-t34;
t36 = qradot.*t35;
t37 = qb+qrh;
t38 = sin(t37);
t39 = Lt.*t38;
t40 = t28+t32-t34+t39;
t41 = qrkdot.*t33;
t42 = qbdot.*t40;
t43 = qrhdot.*t40;
t44 = t36+t41+t42+t43;
t45 = dfmy.*t27;
t46 = dfmx.*t29;
t47 = cos(t30);
t48 = cos(t37);
t71 = Lt.*t48;
t99 = Ls.*t47;
t49 = t45+t46-t71-t99;
t50 = Ltm.*qbdot.*t14;
t51 = Ltm.*qlhdot.*t14;
t52 = t50+t51;
t53 = Ltm.*qbdot.*t38;
t54 = Ltm.*qrhdot.*t38;
t55 = t53+t54;
t56 = Lsm.*t7;
t57 = t15+t56;
t58 = qbdot.*t57;
t59 = qlhdot.*t57;
t60 = Lsm.*qlkdot.*t7;
t61 = t58+t59+t60;
t63 = Lsm.*t23;
t64 = t62+t63;
t65 = Lsm.*t31;
t66 = t39+t65;
t67 = qbdot.*t66;
t68 = qrhdot.*t66;
t69 = Lsm.*qrkdot.*t31;
t70 = t67+t68+t69;
t72 = Lsm.*t47;
t73 = t71+t72;
t74 = sin(qb);
t75 = qbdot.^2;
t76 = cos(qb);
t77 = Lsm.*qlkdot.*t23;
t78 = qbdot.*t64;
t79 = qlhdot.*t64;
t80 = t77+t78+t79;
t81 = Lsm.*qrkdot.*t47;
t82 = qbdot.*t73;
t83 = qrhdot.*t73;
t84 = t81+t82+t83;
t86 = t21+t22;
t87 = t21+t22-t85;
t88 = qlkdot.*t87;
t89 = qladot.*t86;
t90 = qbdot.*t25;
t91 = qlhdot.*t25;
t92 = t88+t89+t90+t91;
t93 = Ltm.*qbdot.*t24;
t94 = Ltm.*qlhdot.*t24;
t95 = t93+t94;
t96 = Ltm.*qbdot.*t48;
t97 = Ltm.*qrhdot.*t48;
t98 = t96+t97;
t100 = t45+t46;
t101 = t45+t46-t99;
t102 = qrkdot.*t101;
t103 = qradot.*t100;
t104 = qbdot.*t49;
t105 = qrhdot.*t49;
t106 = t102+t103+t104+t105;
t107 = qbdot.*t9;
t108 = qlhdot.*t9;
t109 = t12+t17+t107+t108;
t110 = qlkdot.*t109;
t111 = qbdot.*t11;
t112 = qlhdot.*t11;
t113 = qlkdot.*t11;
t114 = t12+t111+t112+t113;
t115 = qladot.*t114;
t116 = qbdot.*t20;
t117 = qlhdot.*t20;
t118 = qladdot.*t86;
t119 = qbddot.*t25;
t120 = qlhddot.*t25;
t121 = Ltm.*qbddot.*t24;
t122 = Ltm.*qlhddot.*t24;
t246 = qbdot.*t52;
t247 = qlhdot.*t52;
t123 = t121+t122-t246-t247+xbddot;
t124 = qbddot.*t64;
t125 = qlhddot.*t64;
t126 = Lsm.*qbdot.*t7;
t127 = Lsm.*qlhdot.*t7;
t128 = t60+t126+t127;
t129 = Lsm.*qlkddot.*t23;
t237 = qbdot.*t61;
t238 = qlhdot.*t61;
t239 = qlkdot.*t128;
t130 = t124+t125+t129-t237-t238-t239+xbddot;
t131 = qbdot.*t33;
t132 = qrhdot.*t33;
t133 = t36+t41+t131+t132;
t134 = qrkdot.*t133;
t135 = qbdot.*t35;
t136 = qrhdot.*t35;
t137 = qrkdot.*t35;
t138 = t36+t135+t136+t137;
t139 = qradot.*t138;
t140 = qbdot.*t44;
t141 = qrhdot.*t44;
t142 = qraddot.*t100;
t143 = qbddot.*t49;
t144 = qrhddot.*t49;
t145 = Ltm.*qbddot.*t48;
t146 = Ltm.*qrhddot.*t48;
t224 = qbdot.*t55;
t225 = qrhdot.*t55;
t147 = t145+t146-t224-t225+xbddot;
t148 = qbddot.*t73;
t149 = qrhddot.*t73;
t150 = Lsm.*qbdot.*t31;
t151 = Lsm.*qrhdot.*t31;
t152 = t69+t150+t151;
t153 = Lsm.*qrkddot.*t47;
t215 = qbdot.*t70;
t216 = qrhdot.*t70;
t217 = qrkdot.*t152;
t154 = t148+t149+t153-t215-t216-t217+xbddot;
t155 = qbddot.*t57;
t156 = qlhddot.*t57;
t157 = Lsm.*qbdot.*t23;
t158 = Lsm.*qlhdot.*t23;
t159 = t77+t157+t158;
t160 = qlkdot.*t159;
t161 = qbdot.*t80;
t162 = qlhdot.*t80;
t163 = Lsm.*qlkddot.*t7;
t164 = t155+t156+t160+t161+t162+t163+ybddot;
t165 = qbddot.*t16;
t166 = qlhddot.*t16;
t167 = qbdot.*t86;
t168 = qlhdot.*t86;
t169 = qlkdot.*t86;
t170 = t89+t167+t168+t169;
t171 = qlkddot.*t9;
t172 = qladdot.*t11;
t173 = qbdot.*t87;
t174 = qlhdot.*t87;
t175 = t88+t89+t173+t174;
t241 = qbdot.*t92;
t242 = qlhdot.*t92;
t243 = qladot.*t170;
t244 = qlkdot.*t175;
t176 = t165+t166+t171+t172-t241-t242-t243-t244+ybddot;
t177 = qbdot.*t95;
t178 = qlhdot.*t95;
t179 = Ltm.*qbddot.*t14;
t180 = Ltm.*qlhddot.*t14;
t181 = t177+t178+t179+t180+ybddot;
t182 = g.*mf;
t183 = g.*ms;
t184 = g.*mt;
t185 = qbddot.*t66;
t186 = qrhddot.*t66;
t187 = Lsm.*qbdot.*t47;
t188 = Lsm.*qrhdot.*t47;
t189 = t81+t187+t188;
t190 = qrkdot.*t189;
t191 = qbdot.*t84;
t192 = qrhdot.*t84;
t193 = Lsm.*qrkddot.*t31;
t194 = t185+t186+t190+t191+t192+t193+ybddot;
t195 = qbdot.*t98;
t196 = qrhdot.*t98;
t197 = Ltm.*qbddot.*t38;
t198 = Ltm.*qrhddot.*t38;
t199 = t195+t196+t197+t198+ybddot;
t200 = qbddot.*t40;
t201 = qrhddot.*t40;
t202 = qbdot.*t100;
t203 = qrhdot.*t100;
t204 = qrkdot.*t100;
t205 = t103+t202+t203+t204;
t206 = qrkddot.*t33;
t207 = qraddot.*t35;
t208 = qbdot.*t101;
t209 = qrhdot.*t101;
t210 = t102+t103+t208+t209;
t219 = qbdot.*t106;
t220 = qrhdot.*t106;
t221 = qradot.*t205;
t222 = qrkdot.*t210;
t211 = t200+t201+t206+t207-t219-t220-t221-t222+ybddot;
t212 = qrkddot.*t101;
t213 = t134+t139+t140+t141+t142+t143+t144+t212-xbddot;
t214 = mf.*t213;
t218 = ms.*t194;
t223 = mf.*t211;
t229 = ms.*t154;
t226 = cts_FRx1+cts_FRx2+cts_FRx3+cts_FRx4+cts_FRx5+t214-t229-mt.*t147;
t227 = mt.*t199;
t228 = -cts_FRy1-cts_FRy2-cts_FRy3-cts_FRy4-cts_FRy5+t182+t183+t184+t218+t223+t227;
t230 = cts_FRy1+cts_FRy2+cts_FRy3+cts_FRy4+cts_FRy5-t182-t183-t218-t223;
t231 = cts_FRx1+cts_FRx2+cts_FRx3+cts_FRx4+cts_FRx5+t214;
t232 = t45+t46-t71-t99+yb;
t233 = cts_FRy1+cts_FRy2+cts_FRy3+cts_FRy4+cts_FRy5-t182-t223;
t234 = qlkddot.*t87;
t235 = t110+t115+t116+t117+t118+t119+t120+t234-xbddot;
t236 = mf.*t235;
t240 = ms.*t164;
t245 = mf.*t176;
t251 = ms.*t130;
t248 = cts_FLx1+cts_FLx2+cts_FLx3+cts_FLx4+cts_FLx5+t236-t251-mt.*t123;
t249 = mt.*t181;
t250 = -cts_FLy1-cts_FLy2-cts_FLy3-cts_FLy4-cts_FLy5+t182+t183+t184+t240+t245+t249;
t252 = cts_FLy1+cts_FLy2+cts_FLy3+cts_FLy4+cts_FLy5-t182-t183-t240-t245;
t253 = cts_FLx1+cts_FLx2+cts_FLx3+cts_FLx4+cts_FLx5+t236;
t254 = t21+t22-t62-t85+yb;
t255 = cts_FLy1+cts_FLy2+cts_FLy3+cts_FLy4+cts_FLy5-t182-t245;
Eq = [cts_FLx1+cts_FLx2+cts_FLx3+cts_FLx4+cts_FLx5+cts_FRx1+cts_FRx2+cts_FRx3+cts_FRx4+cts_FRx5-ms.*t130-ms.*t154-mt.*t123-mt.*t147+mf.*(t110+t115+t116+t117+t118+t119+t120-xbddot+qlkddot.*(t21+t22-Ls.*t23))+mf.*(t134+t139+t140+t141+t142+t143+t144-xbddot+qrkddot.*(t45+t46-Ls.*t47))-mb.*(xbddot-Lbm.*qbddot.*t76+Lbm.*t74.*t75);cts_FLy1+cts_FLy2+cts_FLy3+cts_FLy4+cts_FLy5+cts_FRy1+cts_FRy2+cts_FRy3+cts_FRy4+cts_FRy5+mb.*(-ybddot+Lbm.*qbddot.*t74+Lbm.*t75.*t76)-g.*mb-g.*mf.*2.0-g.*ms.*2.0-g.*mt.*2.0-mf.*t176-mf.*t211-ms.*t164-ms.*t194-mt.*t181-mt.*t199;Tlh+Trh-Ib.*qbddot-Lbm.*t74.*t228+Lbm.*t76.*t226-Lbm.*t74.*t250+Lbm.*t76.*t248;-Trh+Trk+(t71-Ltm.*t48).*(cts_FRx1+cts_FRx2+cts_FRx3+cts_FRx4+cts_FRx5+t214-ms.*t154)+t230.*(t39-Ltm.*t38)-It.*(qbddot+qrhddot)-Ltm.*t38.*t228+Ltm.*t48.*t226;Tra-Trk-Is.*(qbddot+qrhddot+qrkddot)+t233.*(t32-t65)-t231.*(t72-t99)+Lsm.*t31.*t230+Lsm.*t47.*(cts_FRx1+cts_FRx2+cts_FRx3+cts_FRx4+cts_FRx5+t214-t229);-Tra+cts_FRx1.*t232+cts_FRx2.*t232+cts_FRx3.*t232+cts_FRx4.*t232+cts_FRx5.*t232+t35.*t233-t100.*t231-If.*(qbddot+qraddot+qrhddot+qrkddot)-cts_FRy1.*(t28-t34-ctsx1.*t27+ctsy1.*t29)-cts_FRy2.*(t28-t34-ctsx2.*t27+ctsy2.*t29)-cts_FRy3.*(t28-t34-ctsx3.*t27+ctsy3.*t29)-cts_FRy4.*(t28-t34-ctsx4.*t27+ctsy4.*t29)-cts_FRy5.*(t28-t34-ctsx5.*t27+ctsy5.*t29);-Tlh+Tlk+(t62-Ltm.*t24).*(cts_FLx1+cts_FLx2+cts_FLx3+cts_FLx4+cts_FLx5+t236-ms.*t130)+t252.*(t15-Ltm.*t14)-It.*(qbddot+qlhddot)-Ltm.*t14.*t250+Ltm.*t24.*t248;Tla-Tlk-Is.*(qbddot+qlhddot+qlkddot)+t255.*(t8-t56)-t253.*(t63-t85)+Lsm.*t7.*t252+Lsm.*t23.*(cts_FLx1+cts_FLx2+cts_FLx3+cts_FLx4+cts_FLx5+t236-t251);-Tla+cts_FLx1.*t254+cts_FLx2.*t254+cts_FLx3.*t254+cts_FLx4.*t254+cts_FLx5.*t254+t11.*t255-t86.*t253-If.*(qbddot+qladdot+qlhddot+qlkddot)-cts_FLy1.*(t4-t10-ctsx1.*t3+ctsy1.*t5)-cts_FLy2.*(t4-t10-ctsx2.*t3+ctsy2.*t5)-cts_FLy3.*(t4-t10-ctsx3.*t3+ctsy3.*t5)-cts_FLy4.*(t4-t10-ctsx4.*t3+ctsy4.*t5)-cts_FLy5.*(t4-t10-ctsx5.*t3+ctsy5.*t5)];
