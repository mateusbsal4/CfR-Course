function [mu_lr, mu_blr, cov_blr] = param_id(id_data)
    %%-----------------LR ------------------- %%
        u = id_data.input_cur;
        x = id_data.state_cur;
        x_nxt = id_data.state_nxt;
        p = x(1, :);
        v = x(2, :);
        p_nxt = x_nxt(1, :);
        v_nxt = x_nxt(2, :);
        D = length(u);
        Phi = zeros(D, 2);
        Gamma = zeros(D, 2);
        for d=1:D
            Phi(d, 1) = u(d);
            Phi(d, 2) = -cos(3*p(d));
            Gamma(d, 1) = p_nxt(d) - (p(d) + v(d));
            Gamma(d, 2) = v_nxt(d) - v(d);
        end
        theta = inv(((Phi')*Phi))*Phi'*Gamma;
        mu_lr = theta(:, 2);        %both columns should hold the same alpha and beta 
    %---------------------------------------------------------%
    %%------------------------------BLR------------------------%%
        sig = 0.015;
        Sigma_0 = (sig^2)*eye(2)
        mu_0 = zeros(2);
        Sigma_theta_inv = inv(Sigma_0) + (sig^(-2))*Phi'*Phi;
        mu_theta = inv(Sigma_theta_inv)*(inv(Sigma_0)*mu_0 + (sig^(-2))*Phi'*Gamma);
    
        mu_blr = mu_theta(:,2);          %both columns should hold the same alpha and beta 
        cov_blr = inv(Sigma_theta_inv); 
end