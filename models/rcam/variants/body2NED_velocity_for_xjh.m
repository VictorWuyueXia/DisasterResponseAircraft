function vNED=body2NED_velocity(x)
    ub=x(1);
    vb=x(2);
    wb=x(3);
    phi=x(4);
    theta=x(5);
    psi=x(6);
    
    vB=[ub;vb;wb];
    
    cx=@(phi)[1 0 0
        0 cos(phi) sin(phi)
        0 -sin(phi) cos(phi)];
    cy=@(theta)[cos(theta) 0 -sin(theta)
        0 1 0
        sin(theta) 0 cos(theta)];
    
    cz=@(psi)[cos(psi) sin(psi) 0
        -sin(psi) cos(psi) 0
        0 0 1];
    
    C_bv=cz(psi)*cy(theta)*cx(phi);
    C_vb=inv(C_bv);
    
    vNED=C_vb*vB;
    
end
