function [traj_data_array,traj_data_array_home_pose] = trajectory_merger(joint_vec_info,num_joint,num_cyc,freq_rate_hz,home_pos,hold_time)
    
  traj_data_array = [];
  traj_data_array_home_pose = [];
  traj_data_array_forward = [];
  traj_data_array_backward = [];
  
  % ---------------- trajectory points from home pose to initial pose --------------------
  
  if strcmpi(home_pos,'y') == 0
    
    for joint_count = 1:num_joint
      
      %% Initializing array for storing trajectory data
      traj_data_joint = [];
      
      %% Trajectory data for +ve sense of rotation
      theta_initial = 0;
      theta_final = joint_vec_info(joint_count,4);
      time_initial = 0;
      
      if theta_final - theta_initial > 0
        direction = 1;
      else
        direction = -1;
      end
      
      if theta_initial != theta_final
      
        traj_type = 3;
        omega_max = 0.5;
        alpha_max = abs((15/8*omega_max^2/(theta_final-theta_initial)));
        
        % Defining deceleration rate
        beta_max = alpha_max;
        
        %% Initializing array for storing trajectory data
        traj_data_joint = [];
      
        switch traj_type
          case 1
            [time,traj_data]=traj_const_acc(alpha_max,beta_max,omega_max,theta_final,theta_initial,time_initial,direction,freq_rate_hz);
          case 2
            [time,traj_data]=traj_triangular_acc(alpha_max,beta_max,omega_max,theta_final,theta_initial,time_initial,direction,freq_rate_hz);
          case 3
            [time,traj_data]=traj_quintic_vel(alpha_max,beta_max,omega_max,theta_final,theta_initial,time_initial,direction,freq_rate_hz);
          otherwise
            disp('\n Entered value is not a trajectory type')
            break
        end
      
     else
      
        traj_data_joint=[];
        traj_type = 0;
        alpha_max = 0;
        omega_max = 0; 
        % Defining deceleration rate
        beta_max = alpha_max;
        
        if joint_count == 1
          traj_data = [theta_initial zeros(1,2)];  
          time = 0;
        else
          traj_data = [theta_initial*ones(length(traj_data_array_home_pose(:,1)),1) zeros(length(traj_data_array_home_pose(:,1)),2)];  
          time = traj_data_array_home_pose(:,1);
        end
        
        
      end

      %% Storing in trajectory data array
      traj_data_joint = [traj_data_joint; traj_data]; 
      
      if joint_count == 1
        
        traj_data_array_home_pose = [time traj_data_joint];
        row_len_traj_data_array = length (traj_data_array_home_pose(:,1));
        col_len_traj_data_array = length (traj_data_array_home_pose(1,:));
        
      elseif joint_count > 1
      
        row_len_traj_data_array = length (traj_data_array_home_pose(:,1));
        col_len_traj_data_array = length (traj_data_array_home_pose(1,:));
        row_len_traj_data_joint = length (traj_data_joint(:,1));
        
        if row_len_traj_data_joint > row_len_traj_data_array
        
          traj_data_array_home_pose = [traj_data_array_home_pose; zeros(row_len_traj_data_joint-row_len_traj_data_array,col_len_traj_data_array)]; 
          traj_data_array_home_pose(:,1) = time;
          
          for rev_joint_count = 1:joint_count-1
            traj_data_array_home_pose(row_len_traj_data_array+1:row_len_traj_data_joint,3*(rev_joint_count)-1) = traj_data_array_home_pose(row_len_traj_data_array,3*(rev_joint_count)-1)*ones(1:(row_len_traj_data_joint-row_len_traj_data_array-1),1);
          end
          
          traj_data_array_home_pose = [traj_data_array_home_pose traj_data_joint];
        
        elseif row_len_traj_data_joint <= row_len_traj_data_array
          
          traj_data_joint = [traj_data_joint; zeros(row_len_traj_data_array-row_len_traj_data_joint,3)];
          traj_data_joint(row_len_traj_data_joint+1:row_len_traj_data_array) = traj_data_joint(row_len_traj_data_joint,1)*ones(row_len_traj_data_array-row_len_traj_data_joint,1);
          traj_data_array_home_pose = [traj_data_array_home_pose traj_data_joint];
        end 
      end
       
     end
  end
  
  if strcmpi(home_pos,'y') == 0 
    traj_data = ones(freq_rate_hz*hold_time,1)*traj_data_array_home_pose(end,:);
    traj_data(:,1) = ((traj_data_array_home_pose(end,1)+1/freq_rate_hz):1/freq_rate_hz:(traj_data_array_home_pose(end,1)+hold_time));
    traj_data_array_home_pose = [traj_data_array_home_pose; traj_data];
  end
  
  % ----------------- trajectory from initial pose to final pose ------------------------
  
  for joint_count = 1:num_joint
    
    % ---------------- Trajectory data for +ve sense of rotation ---------------------
    
    theta_initial_user = joint_vec_info(joint_count,4);
    theta_final_user = joint_vec_info(joint_count,5);
  
    traj_type = joint_vec_info(joint_count,1);
    alpha_max = joint_vec_info(joint_count,2);
    omega_max = joint_vec_info(joint_count,3);
    
    % Defining deceleration rate
    beta_max = alpha_max;
    
    if strcmpi(home_pos,'y') == 0
      time_initial = traj_data_array_home_pose(end,1)+1/freq_rate_hz;
    else
      time_initial = 0;
    end
    
    if theta_initial_user != theta_final_user
      
      %% Initializing array for storing trajectory data
      traj_data_joint = [];
      
      theta_initial = theta_initial_user;
      theta_final = theta_final_user;
      
      if theta_final - theta_initial > 0
        direction = 1;
      else
        direction = -1;
      end
    
      switch traj_type
        case 1
          [time,traj_data]=traj_const_acc(alpha_max,beta_max,omega_max,theta_final,theta_initial,time_initial,direction,freq_rate_hz);
        case 2
          [time,traj_data]=traj_triangular_acc(alpha_max,beta_max,omega_max,theta_final,theta_initial,time_initial,direction,freq_rate_hz);
        case 3
          [time,traj_data]=traj_quintic_vel(alpha_max,beta_max,omega_max,theta_final,theta_initial,time_initial,direction,freq_rate_hz);
        otherwise
          disp('\n Entered value is not a trajectory type')
          break
      end
    
    else
    
      traj_data_joint=[];
      traj_type = 0;
      alpha_max = 0;
      omega_max = 0; 
      
      if joint_count == 1
        traj_data = [theta_initial_user zeros(1,2)];  
        time = time_initial;
      else
        traj_data = [theta_initial_user*ones(length(traj_data_array_forward(:,1)),1) zeros(length(traj_data_array_forward(:,1)),2)];  
        time = traj_data_array_forward(:,1);
      end
      
      
    end
    
    %% Storing in trajectory data array
    traj_data_joint = [traj_data_joint; traj_data]; 
    
    if joint_count == 1
      
      traj_data_array_forward = [time traj_data_joint];
      row_len_traj_data_array = length (traj_data_array_forward(:,1));
      col_len_traj_data_array = length (traj_data_array_forward(1,:));
      
    elseif joint_count > 1
    
      row_len_traj_data_array = length (traj_data_array_forward(:,1));
      col_len_traj_data_array = length (traj_data_array_forward(1,:));
      row_len_traj_data_joint = length (traj_data_joint(:,1));
      
      if row_len_traj_data_joint > row_len_traj_data_array
      
        traj_data_array_forward = [traj_data_array_forward; zeros(row_len_traj_data_joint-row_len_traj_data_array,col_len_traj_data_array)]; 
        traj_data_array_forward(:,1) = time;
        
        for rev_joint_count = 1:joint_count-1
          traj_data_array_forward(row_len_traj_data_array+1:row_len_traj_data_joint,3*(rev_joint_count)-1) = traj_data_array_forward(row_len_traj_data_array,3*(rev_joint_count)-1)*ones(1:(row_len_traj_data_joint-row_len_traj_data_array-1),1);
        end
        
        traj_data_array_forward = [traj_data_array_forward traj_data_joint];
      
      elseif row_len_traj_data_joint <= row_len_traj_data_array
        
        traj_data_joint = [traj_data_joint; zeros(row_len_traj_data_array-row_len_traj_data_joint,3)];
        traj_data_joint(row_len_traj_data_joint+1:row_len_traj_data_array) = traj_data_joint(row_len_traj_data_joint,1)*ones(row_len_traj_data_array-row_len_traj_data_joint,1);
        traj_data_array_forward = [traj_data_array_forward traj_data_joint];
      
      end
      
    end
     
  end
  
  if abs(hold_time)>0 
    traj_data = ones(freq_rate_hz*hold_time,1)*traj_data_array_forward(end,:);
    traj_data(:,1) = ((traj_data_array_forward(end,1)+1/freq_rate_hz):1/freq_rate_hz:(traj_data_array_forward(end,1)+hold_time));
    traj_data_array_forward = [traj_data_array_forward; traj_data];
  end



  for joint_count =1:num_joint
    
    % ------------------------- Trajectory data for -ve sense of rotation  ------------------------
    
    time_initial = traj_data_array_forward(end,1)+1/freq_rate_hz;    
    traj_type = joint_vec_info(joint_count,1);
    alpha_max = joint_vec_info(joint_count,2);
    omega_max = joint_vec_info(joint_count,3);
    theta_initial_user = joint_vec_info(joint_count,4);
    theta_final_user = joint_vec_info(joint_count,5);
    
    % Defining deceleration rate
    beta_max = alpha_max;
    
    if theta_initial_user != theta_final_user
      
      theta_initial = theta_final_user;
      theta_final = theta_initial_user;
      
      if theta_final - theta_initial > 0
        direction = 1;
      else
        direction = -1;
      end
      
      traj_data_joint = [];
      
      switch traj_type
        case 1
          [time,traj_data]=traj_const_acc(alpha_max,beta_max,omega_max,theta_final,theta_initial,time_initial,direction,freq_rate_hz);
        case 2
          [time,traj_data]=traj_triangular_acc(alpha_max,beta_max,omega_max,theta_final,theta_initial,time_initial,direction,freq_rate_hz);
        case 3
          [time,traj_data]=traj_quintic_vel(alpha_max,beta_max,omega_max,theta_final,theta_initial,time_initial,direction,freq_rate_hz);
        otherwise
          disp('\n Entered value is not a trajectory type')
      end
    
    else
    
      traj_data_joint = [];
      
      if joint_count == 1
        traj_data = [theta_final_user zeros(1,2)];  
        time = time_initial;
      else
        traj_data = [theta_final_user*ones(length(traj_data_array_backward(:,1)),1) zeros(length(traj_data_array_backward(:,1)),2)];  
        time = traj_data_array_backward(:,1);
      end
      
    end

    
    traj_data_joint = [traj_data_joint; traj_data]; 
    
    if joint_count == 1
    
      traj_data_array_backward = [time traj_data_joint];
      row_len_traj_data_array = length (traj_data_array_backward(:,1));
      col_len_traj_data_array = length (traj_data_array_backward(1,:));
      
    elseif joint_count > 1
    
      row_len_traj_data_array = length (traj_data_array_backward(:,1));
      col_len_traj_data_array = length (traj_data_array_backward(1,:));
      row_len_traj_data_joint = length (traj_data_joint(:,1));
      
      if row_len_traj_data_joint > row_len_traj_data_array
      
        traj_data_array_backward = [traj_data_array_backward; zeros(row_len_traj_data_joint-row_len_traj_data_array,col_len_traj_data_array)]; 
        traj_data_array_backward(:,1) = time;
        
        for rev_joint_count = 1:joint_count-1
          traj_data_array_backward(row_len_traj_data_array+1:row_len_traj_data_joint,3*(rev_joint_count)-1) = traj_data_array_backward(row_len_traj_data_array,3*(rev_joint_count)-1)*ones(1:(row_len_traj_data_joint-row_len_traj_data_array-1),1);
        end
        
        traj_data_array_backward = [traj_data_array_backward traj_data_joint];
      
      elseif row_len_traj_data_joint <= row_len_traj_data_array
      
        traj_data_joint = [traj_data_joint; zeros(row_len_traj_data_array-row_len_traj_data_joint,3)];
        traj_data_joint(row_len_traj_data_joint+1:row_len_traj_data_array) = traj_data_joint(row_len_traj_data_joint,1)*ones(row_len_traj_data_array-row_len_traj_data_joint,1);
        traj_data_array_backward = [traj_data_array_backward traj_data_joint];
        
      end
      
    end   
      
  end
  
  if abs(hold_time)>0 
    traj_data = ones(freq_rate_hz*hold_time,1)*traj_data_array_backward(end,:);
    traj_data(:,1) = ((traj_data_array_backward(end,1)+1/freq_rate_hz):1/freq_rate_hz:(traj_data_array_backward(end,1)+hold_time));
    traj_data_array_backward = [traj_data_array_backward; traj_data];
  end

  traj_data_array = [traj_data_array_home_pose; traj_data_array_forward; traj_data_array_backward];
  
  
  if num_cyc > 1
    traj_data_1_cyc = [traj_data_array_forward; traj_data_array_backward];
    len_traj_data_1_cyc = length(traj_data_1_cyc(:,1));
    end_time = traj_data_array(end,1);

    for cnt = 1:num_cyc-1
      traj_data_array = [traj_data_array; traj_data_1_cyc];
      if length(traj_data_array_home_pose) == 0
        traj_data_array(end-len_traj_data_1_cyc+1:end,1) = end_time + 1/freq_rate_hz + traj_data_array(end-len_traj_data_1_cyc+1:end,1);
      else
        traj_data_array(end-len_traj_data_1_cyc+1:end,1) = end_time + 1/freq_rate_hz + traj_data_array(end-len_traj_data_1_cyc+1:end,1) - traj_data_array_home_pose(end,1);
      end
      
      end_time = traj_data_array(end,1);
    end
  end
    
end