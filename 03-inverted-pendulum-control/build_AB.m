function [A,B]=build_AB(M,m,l,D,I,g)
a=M+m
c=m*l
d=I+m*l^2
p=a*d-c^2
A=[0 1 0 0;0 -d*D/p -c*m*g*l/p 0;0 0 0 1;0 c*D/p a*m*g*l/p 0]
B=[0;d/p;0;-c/p]
end
