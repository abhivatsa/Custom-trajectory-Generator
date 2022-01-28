function [time,traj_data]=traj_quintic_vel(alpha_max,beta_max,omega_max,theta_final,theta_initial,time_initial,direction,freq_rate_hz)
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

  % Duration required for reaching maximum velocity from 0
  t_1st_per = 15/8*omega_max/alpha_max;

  % Const velocity time
  t_const_vel = theta_total/omega_max-15/16*omega_max*(1/alpha_max+1/beta_max);

  % Duration required for 0 from reaching maximum velocity
  t_2nd_per = 15/8*omega_max/beta_max;

  % Time instant define in global sense (from t=0 sec)
  t1 = t_1st_per;
  t2 = t_1st_per + t_const_vel;
  t3 = t_1st_per + t_const_vel + t_2nd_per;

  % Total Time of Motion
  tm = t_1st_per + t_const_vel + t_2nd_per;

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
    
    if t < t1
      theta(cnt,1) = theta_initial + (6*omega_max/(15*omega_max/(8*alpha_max))^5)*t^6/6 + (-15*omega_max/(15*omega_max/(8*alpha_max))^4)*t^5/5 + (10*omega_max/(15*omega_max/(8*alpha_max))^3)*t^4/4;
      omega(cnt,1) = (6*omega_max/(15*omega_max/(8*alpha_max))^5)*t^5 + (-15*omega_max/(15*omega_max/(8*alpha_max))^4)*t^4 + (10*omega_max/(15*omega_max/(8*alpha_max))^3)*t^3;
      alpha(cnt,1) = 5*(6*omega_max/(15*omega_max/(8*alpha_max))^5)*t^4 + 4*(-15*omega_max/(15*omega_max/(8*alpha_max))^4)*t^3 + 3*(10*omega_max/(15*omega_max/(8*alpha_max))^3)*t^2;
    end
    
    theta_last = theta_initial + (6*omega_max/(15*omega_max/(8*alpha_max))^5)*t1^6/6 + (-15*omega_max/(15*omega_max/(8*alpha_max))^4)*t1^5/5 + (10*omega_max/(15*omega_max/(8*alpha_max))^3)*t1^4/4;
    omega_last = (6*omega_max/(15*omega_max/(8*alpha_max))^5)*t1^5 + (-15*omega_max/(15*omega_max/(8*alpha_max))^4)*t1^4 + (10*omega_max/(15*omega_max/(8*alpha_max))^3)*t1^3;
    alpha_last = 5*(6*omega_max/t1^5)*t1^4 + 4*(-15*omega_max/t1^4)*t1^3 + 3*(10*omega_max/t1^3)*t1^2;
    
    if t >= t1 && t < t2
      theta(cnt,1) = theta_last+omega_last*(t-t1);
      omega(cnt,1) = omega_last;
      alpha(cnt,1) = 0;
    end
    
    theta_last = theta_last+omega_last*(t2-t1);
    omega_last = omega_last;
    alpha_last = 0;
      
    if t >= t2 && t <= t3
      theta(cnt,1) = -(omega_max/(15*omega_max/(8*beta_max))^5)*(t-t2)^6 + (3*omega_max/(15*omega_max/(8*beta_max))^4)*(t-t2)^5 + (-5/2*omega_max/(15*omega_max/(8*beta_max))^3)*(t-t2)^4 + omega_last*(t-t2) + theta_last;
      omega(cnt,1) = (-6*omega_max/(15*omega_max/(8*beta_max))^5)*(t-t2)^5 + (15*omega_max/(15*omega_max/(8*beta_max))^4)*(t-t2)^4 + (-10*omega_max/(15*omega_max/(8*beta_max))^3)*(t-t2)^3 + omega_last;
      alpha(cnt,1) = 5*(-6*omega_max/(15*omega_max/(8*beta_max))^5)*(t-t2)^4 + 4*(15*omega_max/(15*omega_max/(8*beta_max))^4)*(t-t2)^3 + 3*(-10*omega_max/(15*omega_max/(8*beta_max))^3)*(t-t2)^2;
    end  
    
    theta_last = -(omega_max/(15*omega_max/(8*beta_max))^5)*(t3-t2)^6 + (3*omega_max/(15*omega_max/(8*beta_max))^4)*(t3-t2)^5 + (-5/2*omega_max/(15*omega_max/(8*beta_max))^3)*(t3-t2)^4 + omega_last*(t3-t2) + theta_last;
    omega_last = (-6*omega_max/(15*omega_max/(8*beta_max))^5)*(t3-t2)^5 + (15*omega_max/(15*omega_max/(8*beta_max))^4)*(t3-t2)^4 + (-10*omega_max/(15*omega_max/(8*beta_max))^3)*(t3-t2)^3 + omega_last;
    alpha_last = 5*(-6*omega_max/(15*omega_max/(8*beta_max))^5)*(t3-t2)^4 + 4*(15*omega_max/(15*omega_max/(8*beta_max))^4)*(t3-t2)^3 + 3*(-10*omega_max/(15*omega_max/(8*beta_max))^3)*(t3-t2)^2;

    time(cnt,1) = time_initial + t;
      
  end
  
  time(cnt+1,1) = time(cnt,1) + 1/freq_rate_hz;
  theta(cnt+1,1) = theta_last;
  omega(cnt+1,1) = omega_last;
  alpha(cnt+1,1) = alpha_last;
  
  traj_data = [theta,omega,alpha];


end