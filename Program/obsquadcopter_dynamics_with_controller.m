  function dx = obsquadcopter_dynamics_with_controller(x, a_des)
        dx = zeros(12,1);
        
        pos = x(1:3);
        vel = x(4:6);
        rpy = x(7:9);
        pqr = x(10:12);
        
        roll = rpy(1); pitch = rpy(2); yaw = rpy(3);
        p = pqr(1); q = pqr(2); r = pqr(3);
        
        m = 1;
        g = 10;
        Ix = 0.03;
        Iy = 0.03;
        Iz = 0.05;
        L = 0.2;
        d = 3.13e-3;
        drag = 0.7;
        
        Kp_pos = 2.0;
        Kd_pos = 3.0;
        Kp_att = 100;
        Kd_att = 21;
        Kp_yaw = 0.09;
        Kd_yaw = 0.61;
        
        a_current = vel;
        
        thrust_acc = a_des(3) + drag*vel(3)/m;
        u1 = m * thrust_acc / (cos(roll)*cos(pitch));
        u1 = saturate(u1, 0, 1000);
        
        ax_des = a_des(1) + drag*vel(1)/m;
        ay_des = a_des(2) + drag*vel(2)/m;
        
        pitch_des = atan2(ax_des, g);
        roll_des = atan2(-ay_des*cos(pitch_des), g);
        
        pitch_des = saturate(pitch_des, -pi/6, pi/6);
        roll_des = saturate(roll_des, -pi/6, pi/6);
        
        % Hitung arah horizontal dari akselerasi yang diinginkan
    yaw_des = 0;
    
        
        roll_error = roll_des - roll;
        u2 = Iy/L * (Kp_att*roll_error - Kd_att*p);
        
        pitch_error = pitch_des - pitch;
        u3 = Ix/L * (Kp_att*pitch_error - Kd_att*q);
        
        yaw_error = wrapToPi(yaw_des - yaw);
    u4 = Iz/d * (Kp_yaw*yaw_error - Kd_yaw*r);
    
        
        dx(1:3) = vel;
        
        dx(4) = (sin(yaw)*sin(roll) + cos(yaw)*cos(roll)*sin(pitch))*u1/m - drag*vel(1)/m;
        dx(5) = (-cos(yaw)*sin(roll) + sin(yaw)*cos(roll)*sin(pitch))*u1/m - drag*vel(2)/m;
        dx(6) = -g + cos(roll)*cos(pitch)*u1/m - drag*vel(3)/m;
        
        dx(7) = p + sin(roll)*tan(pitch)*q + cos(roll)*tan(pitch)*r;
        dx(8) = cos(roll)*q - sin(roll)*r;
        dx(9) = sin(roll)/cos(pitch)*q + cos(roll)/cos(pitch)*r;
        
        dx(10) = ((Iy-Iz)*q*r + u2*L)/Ix;
        dx(11) = ((Iz-Ix)*p*r + u3*L)/Iy;
        dx(12) = ((Ix-Iy)*p*q + u4*d)/Iz;
    end
   