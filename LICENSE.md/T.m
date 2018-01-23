function TR=T(a,alpha,d,theta)

TR=[cos(theta) -sin(theta) 0 a;
    sin(theta).*cos(alpha) cos(theta).*cos(alpha) -sin(alpha) -d.*sin(alpha);
    sin(theta).*sin(alpha) cos(theta).*sin(alpha) cos(alpha) d.*cos(alpha);
    0 0 0 1];