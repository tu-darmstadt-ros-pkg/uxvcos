function winkel=zulwin()

LGB=.3;
zul_neigung=.15;
R = 0.1269; 
l_m = 0.2775;    

c=sin(pi/4)*(l_m+R);
d_max=sin(zul_neigung)*c;
beta=0;
g=sin(beta+pi/4)*LGB/2;
r=sin(beta)*(l_m+R);

while g-r>d_max
    g=sin(beta+pi/4)*LGB/2;
    r=sin(beta)*(l_m+R);
    beta_alt=beta;
    beta=beta+0.0001;
end
winkel=beta_alt;
end