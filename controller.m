function u = controller(~, x, params)

    if params.controller == true

        % Extract angles and angular velocities.
        q = x(1:2);
        qdot = x(3:4);

        % Gains and desired state (assumed provided in params)
        Kp = params.Kp;
        Kd = params.Kd;
        qd = params.qd;

        % Compute dynamics terms (assumed to be function handles)
        D = inertia_matrix(q, params) + 1.5*eye(2);
        C = coriolis_matrix(q, qdot, params) + 1.5*eye(2);
        G = gravity_vector(q, params);
        H = C*qdot + G;

        % PD control law (computed torque control) for exponential error convergence
        u = D * (-Kp * (q - qd) - Kd * qdot) + H;
    else 
        u = [0;0];
    end

end