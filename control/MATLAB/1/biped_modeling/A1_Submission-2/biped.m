function xdot = biped(t, x, data)
    % Extract q and qdot from state vector x
    n = length(x) / 2; % Assuming the state vector size is twice the number of joints
    q = x(1:n);
    qdot = x(n+1:end);

    % Extract necessary variables from data structure
    D = data.D(q); % Function handle for D matrix
    C = data.C(q, qdot); % Function handle for C matrix
    G = data.G(q); % Function handle for gradient of potential energy
    B = data.B;
    Kp = data.Kp;
    Kd = data.Kd;
    qref = data.qref;
    H = data.H;

    % Compute the control input tau using the given control law
    % tau = (H*D^(-1)*B)^(-1)*(H*D^(-1)*(C*qdot + G) - Kp*sin(H*q - qref) - Kd*H*qdot)
    tau = inv(H * inv(D) * B) * ( ...
        H * inv(D) * (C * qdot + G) - Kp * sin(H * q - qref) - Kd * H * qdot ...
        );

    % Compute acceleration
    qddot = inv(D) * (- C * qdot - G + B * tau);

    % Form xdot
    xdot = [qdot; qddot];
end


