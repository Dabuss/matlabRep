function out1 = Q_1_3_opt(a,b,c,swivel,x,y,z)
%Q_1_3
%    OUT1 = Q_1_3(A,B,C,SWIVEL,X,Y,Z)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    14-Apr-2017 15:24:46

t3 = sin(a);
t4 = sin(c);
t5 = cos(a);
t6 = cos(c);
t7 = sin(b);
t10 = t4.*t5.*1.52e2;
t11 = t3.*t6.*t7.*1.52e2;
t12 = t10-t11+y;
% t2 = abs(t12);
t14 = t3.*t4.*1.52e2;
t15 = t5.*t6.*t7.*1.52e2;
t16 = t14+t15-x;
% t8 = abs(t16);
% t18 = cos(b);
t19 = t6.*cos(b).*1.52e2;
t20 = t19-z+3.6e2;
% t9 = abs(t20);
% t13 = t12.^2;
% t17 = t16.^2;
% t21 = t20.^2;
t24 = t12.^2+t16.^2+t20.^2;
t22 = t24+1.64e4;
% t33 = t12.*t20;
% t23 = abs(t12.*t20);
% t36 = t16.*t20;
% t25 = abs(t16.*t20);
t27 = 1.0./t24;
t26 = t27.^2;
% t29 = t12.^2;
% t30 = t27.*t12.^2;
% t31 = t16.^2;
% t32 = t27.*t16.^2;
t28 = t27.*t12.^2+t27.*t16.^2;
% t34 = (t12.*t20).^2;
% t35 = t26.*(t12.*t20).^2;
% t37 = (t16.*t20).^2;
% t38 = t26.*(t16.*t20).^2;
% t39 = t28.^2;
% t40 = t26.*(t12.*t20).^2+t26.*(t16.*t20).^2+t28.^2;
t41 = 1.0./sqrt(t26.*(t12.*t20).^2+t26.*(t16.*t20).^2+t28.^2);
% t42 = t22.^2;
% t50 = t27.*t22.^2.*(1.0./4.0);
% t43 = -t27.*t22.^2.*(1.0./4.0)+1.764e5;
t44 = sqrt(-t27.*t22.^2.*(1.0./4.0)+1.764e5);
t45 = sin(swivel);
t46 = 1.0./sqrt(t24);
t47 = t20.^2;
t48 = 1.0./t24.^(3.0./2.0);
t49 = cos(swivel);
% t51 = t12.*t28.*t41.*t46;
% t52 = t12.*t41.*t47.*t48;
% t53 = t12.*t41.*(t28.*t46+t47.*t48);
% t54 = t45.*t12.*t41.*(t28.*t46+t47.*t48);
% t62 = t16.*t20.*t27.*t41.*t49;
t55 = t41.*(t45.*t12.*(t28.*t46+t47.*t48)-t16.*t20.*t27.*t49);
% t56 = t16.*t41.*t47.*t48;
% t57 = t16.*t28.*t41.*t46;
% t58 = t16.*t41.*(t47.*t48+t28.*t46);
% t59 = t45.*t16.*t41.*(t47.*t48+t28.*t46);
% t60 = t12.*t20.*t27.*t41.*t49;
t61 = t41.*(t45.*t16.*(t47.*t48+t28.*t46)+t12.*t20.*t27.*t49);
% t63 = t44.*t55;
% t81 = t16.*t22.*t27.*(1.0./2.0);
t64 = t14+t15+t44.*t55-t16.*t22.*t27.*(1.0./2.0)-x;
% t65 = t44.*t55.*(1.0./4.2e2);
% t83 = t16.*t22.*t27.*(1.0./8.4e2);
t66 = t44.*t55.*(1.0./4.2e2)-t16.*t22.*t27.*(1.0./8.4e2);
% t68 = t20.*t22.*t27.*(1.0./8.4e2);
% t69 = t28.*t41.*t44.*t49.*(1.0./4.2e2);
t70 = t20.*t22.*t27.*(1.0./8.4e2)-t28.*t41.*t44.*t49.*(1.0./4.2e2);
% t77 = t20.*t22.*t27.*(1.0./2.0);
% t78 = t28.*t41.*t44.*t49;
t79 = t19-t20.*t22.*t27.*(1.0./2.0)+t28.*t41.*t44.*t49-z+3.6e2;
% t86 = t64.*t70;
% t87 = t66.*t79;
t88 = t64.*t70+t66.*t79;
% t67 = abs(t88);
% t71 = t44.*t61;
% t72 = t12.*t22.*t27.*(1.0./2.0);
t73 = -t10+t11+t44.*t61+t12.*t22.*t27.*(1.0./2.0)-y;
% t74 = t44.*t61.*(1.0./4.2e2);
% t75 = t12.*t22.*t27.*(1.0./8.4e2);
t76 = t44.*t61.*(1.0./4.2e2)+t12.*t22.*t27.*(1.0./8.4e2);
% t90 = t70.*t73;
% t91 = t76.*t79;
t92 = t70.*t73+t76.*t79;
% t80 = abs(t92);
% t82 = t64.*t76;
% t94 = t66.*t73;
t84 = t64.*t76-t66.*t73;
t85 = abs(t84);
% t89 = t88.^2;
% t93 = t92.^2;
% t95 = t85.^2;
% t96 = t88.^2+t92.^2+t85.^2;
t97 = 1.0./sqrt(t88.^2+t92.^2+t85.^2);
out1 = angle(t84.*t97.*-1i-t66.*t88.*t97-t76.*t92.*t97);