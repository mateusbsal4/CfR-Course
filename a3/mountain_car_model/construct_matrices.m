function [A_bar, B_bar, H_bar] = construct_matrices(x, n_lookahead, dim_state, dim_action)
    % Number of state variables
    state_dim = dim_state;
    % Number of action variables
    action_dim = dim_action;
    
    % Total length of the input vector x
    total_length = n_lookahead * (state_dim + action_dim);
    
    % Initialize A_bar, B_bar, and H_bar as sparse matrices for efficiency
    A_bar = sparse(n_lookahead * state_dim, n_lookahead * state_dim);
    B_bar = sparse(n_lookahead * state_dim, n_lookahead * action_dim);
    H_bar = sparse(n_lookahead * state_dim, state_dim);
    
    % Extract inputs and states from the vector x
    inputs = x(1:n_lookahead * action_dim);
    states = reshape(x(n_lookahead * action_dim + 1:end), state_dim, [])';

    % Construct the matrices
    for i = 1:n_lookahead
        state = states(i, :)';
        input = inputs((i-1) * action_dim + 1 : i * action_dim);
        
        % Get A and B matrices for the current state
        [A, B] = get_lin_matrices(state, input);
        
        % Fill in A_bar
        if i > 1
            A_bar(state_dim * (i-1) + 1:state_dim * i, state_dim * (i-2) + 1:state_dim * (i-1)) = A;
        end
        A_bar(state_dim * (i-1) + 1:state_dim * i, state_dim * (i-1) + 1:state_dim * i) = A;

        % Fill in B_bar
        B_bar(state_dim * (i-1) + 1:state_dim * i, action_dim * (i-1) + 1:action_dim * i) = B;
        
        % Fill in H_bar
        H_bar(state_dim * (i-1) + 1:state_dim * i, :) = eye(state_dim);
    end
end
