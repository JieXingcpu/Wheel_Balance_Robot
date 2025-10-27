%function [ll,theta_l,x_e,y_e,xi,yi]=legFKine(phi1,phi2)
%%五连杆长度，单位m
%l1a=0.075;l2a=l1a;
%l1u=0.12;l2u=l1u;
%l1d=0.2;l2d=l1d;
%syms xe ye
%
%x1=l1a-l1u*cos(phi1);
%y1=l1u*sin(phi1);
%x2=l2a-l2u*cos(phi2);
%y2=l2u*sin(phi2);
%
%eq1=(x1+xe)^2+(y1-ye)^2==l1d^2;
%eq2=(x2-xe)^2+(y2-ye)^2==l2d^2;
%sol=solve([eq1,eq2],[xe,ye]);
%
%xe=sol.xe(1);
%ye=sol.ye(1);
%
%ll=sqrt(xe^2+ye^2);
%theta_l=atan2(xe,ye);
%x_e=xe;
%y_e=ye;
%xi=l1a-l1u*cos(phi1);
%yi=l1u*sin(phi1);
%
%end

%function J = LegJacobian(phi1,phi2,l,theta,xe,ye)
%
%%% 腿长数据
%l1a=0.075;l2a=l1a;
%l1u=0.145;l2u=l1u;
%l1d=0.25;l2d=l1d;
%%% 雅可比矩阵解算
%syms x_1 x_2 y_1 y_2;
%
%x_1=l1a-l1u*cos(phi1);
%x_2=l2a-l2u*cos(phi2);
%y_1=l1u*sin(phi1);
%y_2=l2u*sin(phi2);
%
%J11=1/l1u*...
%    ((xe+x_1)*sin(theta)+(ye-y_1)*cos(theta))/...
%    (-1*(xe+x_1)*sin(phi1)+(ye-y_1)*cos(phi1));
%J12=l/l1u*...
%    ((xe+x_1)*cos(theta)-(ye-y_1)*sin(theta))/...
%    (-1*(xe+x_1)*sin(phi1)+(ye-y_1)*cos(phi1));
%J21=1/l2u*...
%    ((xe-x_2)*sin(theta)+(ye-y_2)*cos(theta))/...
%    ((xe-x_2)*sin(phi2)+(ye-y_2)*cos(phi2));
%J22=l/l2u*...
%    ((xe-x_2)*cos(theta)-(ye-y_2)*sin(theta))/...
%    ((xe-x_2)*sin(phi2)+(ye-y_2)*cos(phi2));
%
%J=inv([J11 J12;J21 J22]);
%
%end