function out1 = Q_1_6(a,b,c,q1,q2,q3,q4)
%Q_1_62
%    OUT1 = Q_1_62(A,B,C,Q1,Q2,Q3,Q4)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    14-Apr-2017 13:51:58

t2 = cos(q1);
t3 = cos(q4);
t4 = sin(q3);
t5 = cos(q2);
t6 = cos(q3);
t7 = sin(q1);
t8 = sin(q2);
t9 = sin(q4);
t10 = cos(a);
t11 = sin(c);
t12 = cos(c);
t13 = sin(a);
t14 = sin(b);
t16 = t11.*t13;
t17 = t10.*t12.*t14;
t18 = t16+t17;
t19 = t10.*t11;
t20 = t12.*t13.*t14;
t21 = t19-t20;
t22 = cos(b);
t24 = t4.*t7;
t25 = t2.*t5.*t6;
t26 = t24-t25;
t27 = t2.*t4;
t28 = t5.*t6.*t7;
t29 = t27+t28;
t15 = t18.*(t3.*t26-t2.*t8.*t9)+t21.*(t3.*t29+t7.*t8.*t9)-t12.*t22.*(t5.*t9-t3.*t6.*t8);
t23 = t18.*(t6.*t7+t2.*t4.*t5)+t21.*(t2.*t6-t4.*t5.*t7)-t4.*t8.*t12.*t22;
out1 = angle(t18.*(t9.*t26+t2.*t3.*t8)+t21.*(t9.*t29-t3.*t7.*t8)+sqrt(t15.^2+t23.^2).*1i+t12.*t22.*(t3.*t5+t6.*t8.*t9));
