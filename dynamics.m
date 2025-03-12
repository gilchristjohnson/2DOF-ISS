function dx = dynamics(~, x, u, params)
    % dynamics  Compute the state derivative for a standard double pendulum.
    %
    %   The dynamics are defined as:
    %       D(q)*q_ddot + C(q,qdot)*qdot + G(q) = B*u,
    %
    %   where the state is:
    %       x = [q1; q2; q1_dot; q2_dot]
    %   and the control input u is 2x1.
    %
    % REFERENCE: https://underactuated.mit.edu/multibody.html
    
    % Extract angles and angular velocities.
    q = x(1:2);
    qdot = x(3:4);
    
    % Compute system matrices using the helper functions.
    D = inertia_matrix(q, params);
    C = coriolis_matrix(q, qdot, params);
    G = gravity_vector(q, params);
    
    % Input matrix (Identity).
    B = eye(2);
    
    % Solve for qddot
    qddot = inv(D)*(B*u - C*qdot - G);
    
    % Assemble state derivative.
    dx = [qdot; qddot];
end