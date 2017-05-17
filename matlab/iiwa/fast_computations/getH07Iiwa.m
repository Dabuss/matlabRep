function out1 = getH07Iiwa(q1,q2,q3,q4,q5,q6,q7)
%GETH07IIWA
%    OUT1 = GETH07IIWA(Q1,Q2,Q3,Q4,Q5,Q6,Q7)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    17-May-2017 10:59:59

t2 = cos(q1);
t3 = cos(q4);
t4 = sin(q1);
t5 = sin(q3);
t6 = t4.*t5;
t7 = cos(q2);
t8 = cos(q3);
t13 = t2.*t7.*t8;
t9 = t6-t13;
t10 = sin(q2);
t11 = sin(q4);
t12 = sin(q5);
t14 = t3.*t9;
t27 = t2.*t10.*t11;
t15 = t14-t27;
t16 = cos(q5);
t17 = t4.*t8;
t18 = t2.*t5.*t7;
t19 = t17+t18;
t20 = sin(q7);
t21 = sin(q6);
t22 = t9.*t11;
t23 = t2.*t3.*t10;
t24 = t22+t23;
t25 = t21.*t24;
t26 = cos(q6);
t28 = t15.*t16;
t29 = t12.*t19;
t30 = t28+t29;
t31 = t26.*t30;
t32 = t25+t31;
t33 = cos(q7);
t34 = t12.*t15;
t35 = t34-t16.*t19;
t36 = t2.*t5;
t37 = t4.*t7.*t8;
t38 = t36+t37;
t39 = t3.*t38;
t40 = t4.*t10.*t11;
t41 = t39+t40;
t42 = t2.*t8;
t48 = t4.*t5.*t7;
t43 = t42-t48;
t44 = t11.*t38;
t55 = t3.*t4.*t10;
t45 = t44-t55;
t46 = t21.*t45;
t47 = t16.*t41;
t49 = t12.*t43;
t50 = t47+t49;
t51 = t26.*t50;
t52 = t46+t51;
t53 = t12.*t41;
t54 = t53-t16.*t43;
t56 = t7.*t11;
t58 = t3.*t8.*t10;
t57 = t56-t58;
t59 = t12.*t57;
t60 = t59-t5.*t10.*t16;
t61 = t16.*t57;
t62 = t5.*t10.*t12;
t63 = t61+t62;
t64 = t3.*t7;
t65 = t8.*t10.*t11;
t66 = t64+t65;
t67 = t21.*t66;
out1 = reshape([t20.*t35-t32.*t33,-t20.*t54+t33.*t52,-t20.*t60-t33.*(t67-t26.*t63),0.0,t20.*t32+t33.*t35,-t20.*t52-t33.*t54,-t33.*t60+t20.*(t67-t26.*t63),0.0,t24.*t26-t21.*t30,t21.*t50-t26.*t45,t21.*t63+t26.*t66,0.0,t2.*t10.*4.2e2+t9.*t11.*4.0e2+t24.*t26.*1.26e2-t21.*t30.*1.26e2+t2.*t3.*t10.*4.0e2,t4.*t10.*4.2e2-t11.*t38.*4.0e2+t21.*t50.*1.26e2-t26.*t45.*1.26e2+t3.*t4.*t10.*4.0e2,t7.*4.2e2+t3.*t7.*4.0e2+t21.*t63.*1.26e2+t26.*t66.*1.26e2+t8.*t10.*t11.*4.0e2+3.6e2,1.0],[4,4]);