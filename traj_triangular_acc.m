function [time,traj_data]=traj_triangular_acc(alpha_max,beta_max,omega_max,theta_final,theta_initial,time_initial,direction,freq_rate_hz)
  %% Output of function:
    % time = Time for which trajectory has been executed
    % traj_data = angular position, velocity, acceleration
    
  %% Input of function:  
    % alpha_max = maximum angular acceleration in rad/sec^2
    % beta_max = maximum angular acceleration in rad/sec^2
    % omega_max = maximum angular velocity in rad/sec
    % theta_final = final angular position of robot in radians
    % theta_initial = final angular position of robot in radians
    % time_initial = time when motion starts in seconds
    % direction = rotation sense of joint
    
  % total rotation of joint (in radians)
  theta_total=abs(theta_final-theta_initial);

  % Duration during which joint is first accelerated and then decelerated (1st triangle)
  t_1st_tri = 2*omega_max/alpha_max;

  % Duration with zero acceleration
  t_const_vel = theta_total/omega_max-omega_max*(1/alpha_max+1/beta_max);

  % Duration during which joint is first decelerated and then accelerated (2nd triangle)
  t_2nd_tri = 2*omega_max/beta_max;

  % Time instant define in global sense (from t=0 sec)
  t1 = t_1st_tri;
  t2 = t_1st_tri + t_const_vel;
  t3 = t_1st_tri + t_const_vel + t_2nd_tri;

  % Total Time of Motion
  tm = t_1st_tri + t_const_vel + t_2nd_tri;
  
  % Updating angular acceleration depending upon direction
  if direction > 0
    alpha_max = alpha_max;
    beta_max = beta_max;
    omega_max = omega_max;
  else
    alpha_max = -alpha_max;
    beta_max = -beta_max;
    omega_max = -omega_max;
  end
  
  cnt=0;  % counter `

  for t = [0:1/freq_rate_hz:floor(tm*freq_rate_hz)/freq_rate_hz]
    
    cnt=cnt+1; 
    
    if t < t1/2
      theta(cnt,1) = theta_initial + alpha_max/(3*t1)*t^3;
      omega(cnt,1) = alpha_max/t1*t^2;
      alpha(cnt,1) = 2*alpha_max/t1*t;
    end
    
    theta_last = theta_initial + alpha_max/(3*t1)*(t1/2)^3;
    omega_last = alpha_max/t1*(t1/2)^2;
    alpha_last = 2*alpha_max/t1*(t1/2);
    
    if t >= t1/2 && t < t1
      theta(cnt,1) = alpha_last/2*(t-t1/2)^2-alpha_max/(3*t1)*(t-t1/2)^3 + omega_last*(t-t1/2) + theta_last;
      omega(cnt,1) = alpha_last*(t-t1/2)-alpha_max/t1*(t-t1/2)^2 + omega_last;
      alpha(cnt,1) = alpha_last-2*alpha_max/t1*(t-t1/2);
    end
    
    theta_last = alpha_last/2*(t1-t1/2)^2-alpha_max/(3*t1)*(t1-t1/2)^3 + omega_last*(t1-t1/2) + theta_last;
    omega_last = alpha_last*(t1-t1/2)-alpha_max/t1*(t1-t1/2)^2 + omega_last;
    alpha_last = alpha_last-2*alpha_max/t1*(t1-t1/2);
      
    if t >= t1 && t < t2
      theta(cnt,1) = theta_last + omega_last*(t-t1);
      omega(cnt,1) = omega_last;
      alpha(cnt,1) = 0;
    end  
    
    theta_last = theta_last + omega_last*(t2-t1);
    omega_last = omega_last;
    alpha_last = 0;
    
    if t >= t2 && t < t2 + (t3-t2)/2
      theta(cnt,1) = -beta_max/(3*t_2nd_tri)*(t-t2)^3 + omega_last*(t-t2) + theta_last;
      omega(cnt,1) = -beta_max/t_2nd_tri*(t-t2)^2+omega_last;
      alpha(cnt,1) = -2*beta_max/t_2nd_tri*(t-t2);
    end
    
    theta_last = -beta_max/(3*t_2nd_tri)*(t2 + (t3-t2)/2 - t2)^3 + omega_last*(t2 + (t3-t2)/2 - t2) + theta_last;
    omega_last = -beta_max/t_2nd_tri*(t2 + (t3-t2)/2 - t2)^2 + omega_last;
    alpha_last = -2*beta_max/t_2nd_tri*(t2 + (t3-t2)/2 - t2);
    
    if t >= t2 + (t3-t2)/2 && t <= t3
      theta(cnt,1) = -beta_max*(t-(t2 + t_2nd_tri/2))^2/2 + beta_max*(t-(t2 + t_2nd_tri/2))^3/(3*t_2nd_tri) + omega_last*(t-(t2 + t_2nd_tri/2)) + theta_last;
      omega(cnt,1) = -beta_max*(t-(t2 + t_2nd_tri/2)) + beta_max/t_2nd_tri*(t-(t2 + t_2nd_tri/2))^2+omega_last;
      alpha(cnt,1) = -beta_max + 2*beta_max/t_2nd_tri*(t-(t2 + t_2nd_tri/2));
    end
    
    theta_last = -beta_max*(t3-(t2 + t_2nd_tri/2))^2/2 + beta_max*(t3-(t2 + t_2nd_tri/2))^3/(3*t_2nd_tri) + omega_last*(t3-(t2 + t_2nd_tri/2)) + theta_last;
    omega_last = -beta_max*(t3-(t2 + t_2nd_tri/2)) + beta_max/t_2nd_tri*(t3-(t2 + t_2nd_tri/2))^2+omega_last;
    alpha_last = -beta_max + 2*beta_max/t_2nd_tri*(t3-(t2 + t_2nd_tri/2));
    
    time(cnt,1) = time_initial + t;
      
  end
  
  time(cnt+1,1) = time(cnt,1) + 1/freq_rate_hz;
  theta(cnt+1,1) = theta_last;
  omega(cnt+1,1) = omega_last;
  alpha(cnt+1,1) = alpha_last;

  traj_data = [theta,omega,alpha];

end