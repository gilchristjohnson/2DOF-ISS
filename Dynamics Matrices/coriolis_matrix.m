function C = coriolis_matrix(q, qdot, params)
    % coriolis_matrix  Computes the Coriolis matrix C(q,qdot) for the double pendulum.
    %
    %   q = [q1; q2],   qdot = [q1_dot; q2_dot]
    %   params should contain: m2, l1, l2.
    %
    % REFERENCE: https://underactuated.mit.edu/multibody.html
    
    m2 = params.m2;
    l1 = params.l1;
    l2 = params.l2;
    
    C11 = 0;
    C12 = -m2 * l1 * l2 * (2*qdot(1) + qdot(2)) * sin(q(2));
    C21 = 0.5 * m2 * l1 * l2 * (2*qdot(1) + qdot(2)) * sin(q(2));
    C22 = -0.5 * m2 * l1 * l2 * qdot(1) * sin(q(2));
    
    C = [C11, C12;
         C21, C22];
end