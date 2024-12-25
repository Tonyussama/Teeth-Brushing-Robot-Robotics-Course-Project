
function Task_Space = task_traj3(X0, Xf, Tf, Ts)
    Task_Space = [];
    
    i = 0;
    count = 0;
    m=0;
    while(i <= Tf)
        count = count+1;
        x = 0.05 * cos((pi) * i)-0.02;
        y = X0(2);
        z = 0.05 * sin((pi) * i)+0.078+m; 
      
      
        % (~mod(count,10))*(count/10)*0.01;
        % Add additional height condition when t is even
        if mod(count,10)==0
            m= m+0.005 ;
            fprintf("hello");

        end
        
        Vec = [x, y, z];
        Task_Space = [Task_Space; Vec];
        i = i + Ts;
        
    end

    % Plotting the trajectory in 3D space
    % plot3((0:Ts:Tf),Task_Space(:, 1), Task_Space(:, 3)); % Assuming Task_Space contains x, y, z coordinates
    % xlabel('Time');
    % ylabel('X-axis');
    % zlabel('Z-axis');
    % title('3D Trajectory Plot');
      % Plotting x and z coordinates
    plot(Task_Space(:, 3),Task_Space(:, 1)); % Plot x vs z
    xlabel('Z-axis');
    ylabel('X-axis');
    title('2D Plot of X and Z Coordinates');
    % plot(Task_Space(:, 3),(0:Ts:Tf)); % Plot x vs z
    % xlabel('Z-axis');
    % ylabel('TIME');
    % title('2D Plot of X and Z Coordinates');
    q_0=[pi/6,pi/6,pi/6,pi/6]';
    q=[0,0,0,0];
    for i=2:51
    q(i,:)=vpa(inverse_kinematics_func(q_0,Task_Space(i,:))',2);
    end
    q=[q; 0 0 0 0];
    % Save 'q' to the MATLAB workspace
    q
    assignin('base', 'q_des_orig', q); % Replace 'q_variable_name' with your desired variable name
end
