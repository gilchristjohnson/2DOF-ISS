function D = inertia_matrix(q, params)
    % inertia_matrix  Computes the inertia matrix D(q) for the double pendulum.
    %
    %   q = [q1; q2]
    %   params should contain: m1, m2, l1, l2.
    %
    % REFERENCE: https://underactuated.mit.edu/multibody.html
    
    m1 = params.m1;
    m2 = params.m2;
    l1 = params.l1;
    l2 = params.l2;
    
    D11 = (m1 + m2)*l1^2 + m2*l2^2 + 2*m2*l1*l2*cos(q(2));
    D12 = m2*l2^2 + m2*l1*l2*cos(q(2));
    D21 = D12;
    D22 = m2*l2^2;
    
    D = [D11, D12;
         D21, D22];
end