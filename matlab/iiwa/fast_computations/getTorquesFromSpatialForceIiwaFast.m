function Tfsym = getTorquesFromSpatialForceIiwaFast(config, tcpEulerZYX, f)
%GETTORQUESFROMSPATIALFORCEIIWAFAST1
%    TFSYM = GETTORQUESFROMSPATIALFORCEIIWAFAST1(FROTX,FROTY,FROTZ,FX,FY,FZ,Q1,Q2,Q3,Q4,Q5,Q6,Q7,XEE,YEE,ZEE)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    17-May-2017 11:50:55

% generated thanks to function getTorquesFromspatialForceIiwa

q1 = config(1); q2 = config(2); q3 = config(3); q4 = config(4); q5 = config(5); q6 = config(6); q7 = config(7); 
xEE = tcpEulerZYX(1); yEE = tcpEulerZYX(2); zEE = tcpEulerZYX(3); aEE = tcpEulerZYX(4); bEE = tcpEulerZYX(5); cEE = tcpEulerZYX(6);
fx = f(1); fy = f(2); fz = f(3); frotx = f(4); froty = f(5); frotz = f(6); 


t2 = sin(q1);
t3 = cos(q4);
t4 = cos(q1);
t5 = sin(q3);
t6 = t4.*t5;
t7 = cos(q2);
t8 = cos(q3);
t9 = t2.*t7.*t8;
t10 = t6+t9;
t11 = sin(q2);
t12 = sin(q4);
t13 = sin(q5);
t14 = t3.*t10;
t15 = t2.*t11.*t12;
t16 = t14+t15;
t17 = cos(q5);
t18 = t4.*t8;
t27 = t2.*t5.*t7;
t19 = t18-t27;
t20 = sin(q7);
t21 = sin(q6);
t22 = t10.*t12;
t35 = t2.*t3.*t11;
t23 = t22-t35;
t24 = t21.*t23;
t25 = cos(q6);
t26 = t16.*t17;
t28 = t13.*t19;
t29 = t26+t28;
t30 = t25.*t29;
t31 = t24+t30;
t32 = cos(q7);
t33 = t13.*t16;
t56 = t17.*t19;
t34 = t33-t56;
t36 = t2.*t5;
t38 = t4.*t7.*t8;
t37 = t36-t38;
t39 = t12.*t37;
t40 = t3.*t4.*t11;
t41 = t39+t40;
t42 = t3.*t37;
t50 = t4.*t11.*t12;
t43 = t42-t50;
t44 = t17.*t43;
t45 = t2.*t8;
t46 = t4.*t5.*t7;
t47 = t45+t46;
t48 = t13.*t47;
t49 = t44+t48;
t51 = t21.*t41;
t52 = t25.*t49;
t53 = t51+t52;
t54 = t13.*t43;
t76 = t17.*t47;
t55 = t54-t76;
t57 = t20.*t34;
t118 = t31.*t32;
t58 = t57-t118;
t59 = t58.*xEE.*(1.0./1.0e3);
t60 = t20.*t31;
t61 = t32.*t34;
t62 = t60+t61;
t63 = t62.*yEE.*(1.0./1.0e3);
t64 = t23.*t25.*(6.3e1./5.0e2);
t65 = t10.*t12.*(2.0./5.0);
t66 = t23.*t25;
t120 = t21.*t29;
t67 = t66-t120;
t68 = t67.*zEE.*(1.0./1.0e3);
t119 = t21.*t29.*(6.3e1./5.0e2);
t121 = t2.*t3.*t11.*(2.0./5.0);
t69 = t59+t63+t64+t65+t68-t119-t121-t2.*t11.*(2.1e1./5.0e1);
t70 = t4.*t11.*(2.1e1./5.0e1);
t71 = t25.*t41.*(6.3e1./5.0e2);
t72 = t12.*t37.*(2.0./5.0);
t73 = t25.*t41;
t116 = t21.*t49;
t74 = t73-t116;
t75 = t74.*zEE.*(1.0./1.0e3);
t77 = t20.*t55;
t117 = t32.*t53;
t78 = xEE.*(t77-t117).*(1.0./1.0e3);
t79 = t20.*t53;
t80 = t32.*t55;
t81 = t79+t80;
t82 = t81.*yEE.*(1.0./1.0e3);
t83 = t3.*t4.*t11.*(2.0./5.0);
t115 = t21.*t49.*(6.3e1./5.0e2);
t84 = t70+t71+t72+t75+t78+t82+t83-t115;
t85 = t7.*t12;
t93 = t3.*t8.*t11;
t86 = t85-t93;
t87 = t17.*t86;
t88 = t5.*t11.*t13;
t89 = t87+t88;
t90 = t3.*t7;
t91 = t8.*t11.*t12;
t92 = t90+t91;
t94 = t13.*t86;
t106 = t5.*t11.*t17;
t95 = t94-t106;
t96 = t21.*t92;
t108 = t25.*t89;
t97 = t96-t108;
t98 = t7.*(2.1e1./5.0e1);
t99 = t3.*t7.*(2.0./5.0);
t100 = t21.*t89;
t101 = t25.*t92;
t102 = t100+t101;
t103 = t102.*zEE.*(1.0./1.0e3);
t104 = t21.*t89.*(6.3e1./5.0e2);
t105 = t25.*t92.*(6.3e1./5.0e2);
t107 = t20.*t95;
t109 = t32.*t97;
t110 = t107+t109;
t111 = t20.*t97;
t123 = t32.*t95;
t131 = t111-t123;
t112 = t131.*yEE.*(1.0./1.0e3);
t113 = t8.*t11.*t12.*(2.0./5.0);
t122 = t110.*xEE.*(1.0./1.0e3);
t114 = t98+t99+t103+t104+t105+t112+t113-t122;
t124 = t71+t72+t75+t78+t82+t83-t115;
t125 = t99+t103+t104+t105+t112+t113-t122;
t126 = xEE.*(t57-t118).*(1.0./1.0e3);
t127 = t63+t64+t65+t68-t119-t121+t126;
t128 = t71+t75+t78+t82-t115;
t129 = t103+t104+t105+t112-t122;
t130 = t63+t64+t68-t119+t126;
t132 = t63+t68+t126;
t133 = t75+t78+t82;
t134 = t103+t112-t122;
Tfsym = [frotz+fx.*t69+fy.*t84;-frotx.*t2+froty.*t4+fz.*(t2.*t69-t4.*t84)+fx.*t4.*t114+fy.*t2.*t114;fx.*(t7.*t127+t2.*t11.*t125)+fy.*(t7.*t124-t4.*t11.*t125)+frotz.*t7-fz.*(t4.*t11.*(t59+t63+t64+t65+t68-t119-t121)+t2.*t11.*t124)+froty.*t2.*t11+frotx.*t4.*t11;-fx.*(t19.*t125+t5.*t11.*t127)-fy.*(t47.*t125+t5.*t11.*t124)-froty.*t19+frotx.*t47+fz.*(t19.*t124-t47.*t127)-frotz.*t5.*t11;-froty.*t23+frotx.*t41+frotz.*t92-fx.*(t23.*t129-t92.*t130)-fy.*(t41.*t129-t92.*t128)+fz.*(t23.*t128-t41.*t130);-froty.*t34+frotx.*t55-frotz.*t95-fx.*(t34.*t129+t95.*t130)-fy.*(t55.*t129+t95.*t128)+fz.*(t34.*t128-t55.*t130);-froty.*t67+frotx.*t74+frotz.*t102-fx.*(t67.*t134-t102.*t132)-fy.*(t74.*t134-t102.*t133)+fz.*(t67.*t133-t74.*t132)];
