function out1 = Q_1_5(a,b,c,swivel,x,y,z)
%Q_1_5
%    OUT1 = Q_1_5(A,B,C,SWIVEL,X,Y,Z)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    18-Apr-2017 10:26:14

t2 = cos(a);
t3 = sin(c);
t4 = cos(c);
t5 = sin(a);
t6 = sin(b);
t10 = t2.*t3.*1.52e2;
t11 = t4.*t5.*t6.*1.52e2;
t12 = t10-t11+y;
t7 = abs(t12);
t14 = t3.*t5.*1.52e2;
t15 = t2.*t4.*t6.*1.52e2;
t16 = t14+t15-x;
t8 = abs(t16);
t18 = cos(b);
t19 = t4.*t18.*1.52e2;
t20 = t19-z+3.6e2;
t9 = abs(t20);
t13 = t7.^2;
t17 = t8.^2;
t21 = t9.^2;
t22 = t13+t17+t21+1.64e4;
t23 = t13+t17+t21;
t24 = 1.0./t23;
t33 = t12.*t20;
t25 = abs(t33);
t36 = t16.*t20;
t26 = abs(t36);
t27 = 1.0./t23.^2;
t28 = t12.^2;
t29 = t24.*t28;
t30 = t16.^2;
t31 = t24.*t30;
t32 = t29+t31;
t34 = t25.^2;
t35 = t27.*t34;
t37 = t26.^2;
t38 = t27.*t37;
t39 = t32.^2;
t40 = t35+t38+t39;
t41 = 1.0./sqrt(t40);
t42 = t22.^2;
t51 = t24.*t42.*(1.0./4.0);
t43 = -t51+1.764e5;
t44 = sqrt(t43);
t45 = sin(swivel);
t46 = t20.^2;
t47 = 1.0./t23.^(3.0./2.0);
t48 = 1.0./sqrt(t23);
t49 = cos(swivel);
t66 = t16.*t41.*t46.*t47;
t67 = t16.*t32.*t41.*t48;
t68 = t66+t67;
t69 = t45.*t68;
t70 = t12.*t20.*t24.*t41.*t49;
t71 = t69+t70;
t72 = t44.*t71;
t73 = t12.*t22.*t24.*(1.0./2.0);
t76 = t72+t73;
t50 = abs(t76);
t52 = t12.*t32.*t41.*t48;
t53 = t12.*t41.*t46.*t47;
t54 = t52+t53;
t55 = t45.*t54;
t62 = t16.*t20.*t24.*t41.*t49;
t56 = t55-t62;
t63 = t44.*t56;
t64 = t16.*t22.*t24.*(1.0./2.0);
t78 = t63-t64;
t57 = abs(t78);
t59 = t20.*t22.*t24.*(1.0./2.0);
t60 = t32.*t41.*t44.*t49;
t75 = t59-t60;
t58 = abs(t75);
t82 = t19-t59+t60-z+3.6e2;
t61 = abs(t82);
t79 = t14+t15+t63-t64-x;
t65 = abs(t79);
t81 = -t10+t11+t72+t73-y;
t74 = abs(t81);
t77 = conj(t44);
t80 = t71.*t77;
t118 = t32.*t41.*t49.*t77;
t119 = t73+t80;
t120 = t76.*t119;
t121 = t56.*t77;
t122 = t14+t15-t64+t121-x;
t123 = t79.*t122;
t124 = -t10+t11+t73+t80-y;
t125 = t81.*t124;
t126 = t19-t59+t118-z+3.6e2;
t127 = t82.*t126;
t83 = -t28-t30-t46+t120+t123+t125+t127+t75.*(t59-t32.*t41.*t49.*t77)-t78.*(t64-t56.*t77);
t84 = t44.*t71.*(1.0./4.2e2);
t85 = t12.*t22.*t24.*(1.0./8.4e2);
t86 = t84+t85;
t87 = t44.*t56.*(1.0./4.2e2);
t89 = t16.*t22.*t24.*(1.0./8.4e2);
t88 = t87-t89;
t91 = t20.*t22.*t24.*(1.0./8.4e2);
t92 = t32.*t41.*t44.*t49.*(1.0./4.2e2);
t93 = t91-t92;
t98 = t79.*t93;
t99 = t82.*t88;
t100 = t98+t99;
t90 = abs(t100);
t102 = t81.*t93;
t103 = t82.*t86;
t104 = t102+t103;
t94 = abs(t104);
t95 = t79.*t86;
t106 = t81.*t88;
t96 = t95-t106;
t97 = abs(t96);
t101 = t90.^2;
t105 = t94.^2;
t107 = t97.^2;
t108 = t101+t105+t107;
t109 = 1.0./sqrt(t108);
t110 = t50.^2;
t111 = t57.^2;
t112 = t58.^2;
t113 = t110+t111+t112;
t114 = t61.^2;
t115 = t65.^2;
t116 = t74.^2;
t117 = t114+t115+t116;
t128 = 1.0./t113;
t129 = 1.0./t117;
t130 = t59-t118;
t131 = t75.*t130;
t132 = t64-t121;
t136 = t78.*t132;
t133 = -t28-t30-t46+t120+t123+t125+t127+t131-t136;
t134 = 1.0./sqrt(t113);
t135 = 1.0./sqrt(t117);
t137 = t133.^2;
t138 = t128.*t129.*t137.*(-1.0./4.0)+1.0;
t139 = sqrt(t138);
t140 = t2.*t3;
t141 = t140-t4.*t5.*t6;
t142 = t3.*t5;
t143 = t2.*t4.*t6;
t144 = t142+t143;
out1 = angle(-t141.*(t86.*t139+t133.*t134.*t135.*(t88.*t96.*t109-t93.*t104.*t109).*(1.0./2.0))+t144.*(t88.*sqrt(t83.^2.*t128.*t129.*(-1.0./4.0)+1.0)-t133.*t134.*t135.*(t86.*t96.*t109+t93.*t100.*t109).*(1.0./2.0))-t4.*t18.*(t93.*t139+t133.*t134.*t135.*(t88.*t100.*t109+t86.*t104.*t109).*(1.0./2.0))+t100.*t109.*t141.*1i+t104.*t109.*t144.*1i-t4.*t18.*t96.*t109.*1i);
