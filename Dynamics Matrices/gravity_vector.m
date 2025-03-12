function G = gravity_vector(q, params)
    % gravity_vector  Computes the gravity vector G(q) for the double pendulum.
    %
    %   q = [q1; q2]
    %   params should contain: m1, m2, l1, l2, g.
    %
    % REFERENCE: https://underactuated.mit.edu/multibody.html
    
    m1 = params.m1;
    m2 = params.m2;
    l1 = params.l1;
    l2 = params.l2;
    g  = params.g;
    
    G1 = (m1 + m2)*l1*sin(q(1)) + m2*l2*sin(q(1) + q(2));
    G2 = m2*l2*sin(q(1) + q(2));
    
    G = g*[G1; G2];
end