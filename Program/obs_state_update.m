   function x = obs_state_update(x, a_des, dt)
        g = 10;     % gravity (m/s^2)
        
        
        a_des(3) = a_des(3) + g;
        
        dx = obsquadcopter_dynamics_with_controller(x, a_des);
        x = x + dt*dx;
    end
    