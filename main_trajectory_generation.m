clc;
clear all;
close all;

pkg load mapping

num_joint = input('Enter the number of joints : ');
num_cyc = input('Enter the number of cycles : ');
freq_rate_hz = input('Enter the frequency for data generation (in Hz) : ');

home_pos = input("\n Start robot from home pose (y/n): ",'s');

hold_time = input('Enter the hold time after reaching joint limits (in sec) : ');

joint_vec_info = [];

% ------------------------- if starting pose is home pose --------------------------
if strcmpi(home_pos,'y') == 1
  
  for joint_count = 1:num_joint
    
    % ----------------------- start and final position of different joints -------------------- 
    
    fprintf("\n");
    theta_initial_user = 0;
    theta_final_user = deg2rad(input(strcat('Provide final position of Joint - 0', num2str(joint_count), ' in degree : ')));
    
    % ---------------- if start position and end position are different ------------------------
    
    if theta_initial_user != theta_final_user
    
      % -------------------  Feeding maximum joint velocity input from user -----------------------
      omega_max = deg2rad(input(strcat('Provide maximum angular speed of Joint - 0', num2str(joint_count), ' in degree/sec : ')));
      
      min_alpha_const_acc = (omega_max^2/(theta_final_user-theta_initial_user))*180/pi;
      min_alpha_tri_acc = (2*omega_max^2/(theta_final_user-theta_initial_user))*180/pi;
      min_alpha_quintic_vel = (15/8*omega_max^2/(theta_final_user-theta_initial_user))*180/pi;
      
      % ------------------ selection of Trajectory type for joint rotation  -----------------------
      
      fprintf(strcat("Enter the type of trajectory \n     1.      Const acc ( min acc value : ", num2str(min_alpha_const_acc), " ) \n     2.      Triangular acc ( min acc value : ", num2str(min_alpha_tri_acc), " ) \n     3.      quintic vel ( min acc value : ", num2str(min_alpha_quintic_vel), " ) "));
      traj_type = input("\n Please enter the associated number : ");
      
      % --------------------  Feeding maximum joint acceleration input from user  ---------------------
      alpha_max = deg2rad(input(strcat('Provide maximum angular acceleration of Joint - 0', num2str(joint_count), ' in degree/sec^2 : ')));
      
    else
    
      traj_type = 0;
      alpha_max = 0;
      omega_max = 0;      
      
    end
    
    joint_vec_info = [joint_vec_info; traj_type alpha_max omega_max theta_initial_user theta_final_user];
    
  end
  
  
% ------------------------- if starting pose is different than home pose --------------------------
else
  
  for joint_count = 1:num_joint
  
    % ----------------------- start and final position of different joints -------------------- 
    
    fprintf("\n");
    theta_initial_user = deg2rad(input(strcat('Provide initial position of Joint - 0', num2str(joint_count), ' in degree : ')));
    theta_final_user = deg2rad(input(strcat('Provide final position of Joint - 0', num2str(joint_count), ' in degree : ')));
    
    % ---------------- if start position and end position are different ------------------------
    if theta_initial_user != theta_final_user
    
      % ------------------ Feeding maximum joint velocity input from user ---------------------
      omega_max = deg2rad(input(strcat('Provide maximum angular speed of Joint - 0', num2str(joint_count), ' in degree/sec : ')));
      
      min_alpha_const_acc = (omega_max^2/(theta_final_user-theta_initial_user))*180/pi;
      min_alpha_tri_acc = (2*omega_max^2/(theta_final_user-theta_initial_user))*180/pi;
      min_alpha_quintic_vel = (15/8*omega_max^2/(theta_final_user-theta_initial_user))*180/pi;
      
      % ----------------------- selection of Trajectory type for joint rotation  ------------------ 
      
      fprintf(strcat("Enter the type of trajectory \n     1.      Const acc ( min acc value : ", num2str(min_alpha_const_acc), " ) \n     2.      Triangular acc ( min acc value : ", num2str(min_alpha_tri_acc), " ) \n     3.      quintic vel ( min acc value : ", num2str(min_alpha_quintic_vel), " ) "));
      traj_type = input("\n Please enter the associated number : ");
      
      % ----------------------  Feeding maximum joint acceleration input from user  ----------------
      alpha_max = deg2rad(input(strcat('Provide maximum angular acceleration of Joint - 0', num2str(joint_count), ' in degree/sec^2 : ')));
      
    else
    
      traj_type = 0;
      alpha_max = 0;
      omega_max = 0; 
      
    end
    
    joint_vec_info = [joint_vec_info; traj_type alpha_max omega_max theta_initial_user theta_final_user];
    
  end
end

[traj_data_array,thp] = trajectory_merger(joint_vec_info,num_joint,num_cyc,freq_rate_hz,home_pos,hold_time);

csv_write = input("\n Store data in CSV file (y/n): ",'s');

if strcmpi(csv_write,'y') == 1
  file_name = input("\n Enter the file name : ",'s');
  csvwrite(strcat("Custom_",file_name,".csv"),traj_data_array);
end