function out1 = Q_1_2(a,b,c,swivel,x,y,z)
%Q_1_2
%    OUT1 = Q_1_2(A,B,C,SWIVEL,X,Y,Z)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    14-Apr-2017 08:41:49

t3 = sin(a);
t4 = sin(c);
t5 = cos(a);
t6 = cos(c);
t7 = sin(b);
t10 = t4.*t5.*1.52e2;
t11 = t3.*t6.*t7.*1.52e2;
t12 = t10-t11+y;
t2 = abs(t12);
t14 = t3.*t4.*1.52e2;
t15 = t5.*t6.*t7.*1.52e2;
t16 = t14+t15-x;
t8 = abs(t16);
t18 = cos(b);
t19 = t6.*t18.*1.52e2;
t20 = t19-z+3.6e2;
t9 = abs(t20);
t13 = t2.^2;
t17 = t8.^2;
t21 = t9.^2;
t22 = t13+t17+t21+1.64e4;
t33 = t12.*t20;
t23 = abs(t33);
t24 = t13+t17+t21;
t36 = t16.*t20;
t25 = abs(t36);
t26 = 1.0./t24.^2;
t27 = 1.0./t24;
t29 = t12.^2;
t30 = t27.*t29;
t31 = t16.^2;
t32 = t27.*t31;
t28 = t30+t32;
t34 = t23.^2;
t35 = t26.*t34;
t37 = t25.^2;
t38 = t26.*t37;
t39 = t28.^2;
t40 = t35+t38+t39;
t41 = 1.0./sqrt(t40);
t43 = t22.^2;
t44 = t27.*t43.*(1.0./4.0);
t45 = -t44+1.764e5;
t46 = sqrt(t45);
t47 = sin(swivel);
t48 = 1.0./sqrt(t24);
t49 = t20.^2;
t50 = 1.0./t24.^(3.0./2.0);
t51 = cos(swivel);
t42 = t46.*(t47.*(t16.*t28.*t41.*t48+t16.*t41.*t49.*t50)+t12.*t20.*t27.*t41.*t51)+t12.*t22.*t27.*(1.0./2.0);
t52 = t46.*(t47.*(t12.*t28.*t41.*t48+t12.*t41.*t49.*t50)-t16.*t20.*t27.*t41.*t51)-t16.*t22.*t27.*(1.0./2.0);
out1 = angle(sqrt(t42.^2+t52.^2).*1i-t20.*t22.*t27.*(1.0./2.0)+t28.*t41.*t46.*t51);
