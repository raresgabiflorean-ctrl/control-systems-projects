function dz=pid_rhs(t,z,A,B,Kp,Ki,Kd,Fmax,antiwindup)
q=z(1:4)
xi=z(5)
theta=q(3)
thetadot=q(4)
e=-theta
edot=-thetadot
u_unsat=Kp*e+Ki*xi+Kd*edot
u_sat=max(min(u_unsat,Fmax),-Fmax)
dq=A*q+B*u_sat
if antiwindup && (u_unsat ~= u_sat)
    dxi=0
else
    dxi=e
end
dz=[dq;dxi]
end