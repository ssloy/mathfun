function funHandle = GetHandleRWode(params)
%RWODE Compute the derivatives


% The parameters are: J, Jr, Cp, Cr, R, k, Umax, mlg

    J = params(1);
    Jr = params(2);
    Cp = params(3);
    Cr = params(4);
    R = params(5);
    k = params(6);
    Umax = params(7);
    mlg = params(8);

% The constant model matrices are    
    M = [Jr Jr; Jr J+Jr];
    C = [Cr+k^2/R  0; 0 Cp];  %friction and back-EMF
    T = -[k/R; 0]*Umax;

% Now we return the handle for ode computations
    funHandle = @(t,x) RWode(t,x);
    
%% This is the ode function 
    function dxdt = RWode(~,x)

        % The state is x=[q;dq];
        q=x(1:2); %(theta, theta_r)
        dq=x(3:4); %(dot{theta}, dot{theta_r})
        
        % the state-dependednt matrix G
        G = [0; -mlg*sin(q(2))];
        
        % the second derivative is 
        d2q = M\(-G-C*dq+T);
        
        % the output
        dxdt=[dq; d2q];
    end
end

