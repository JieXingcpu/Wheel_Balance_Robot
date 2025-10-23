function result=state_calc(theta_wl,dtheta_wl,theta_wr,dtheta_wr)

R_w=0.0425;
s=R_w*(theta_wl+theta_wr)/2;
s_dot=R_w*(dtheta_wl+dtheta_wr)/2;



result=[s;s_dot];

end