function [InitialObservation, LoggedSignal] = rest_RLp()
% Reset function to place custom cart-pole environment into a random
% initial state.

LoggedSignal.Par.risWin = 4;
LoggedSignal.Par.N = [16, 32];
LoggedSignal.Par.sigma = -110; % dBm
LoggedSignal.Par.P = 35; % dBm
LoggedSignal.Par.beta_0 = -40; % dB
LoggedSignal.Par.carier_frequence = 2.4; % GHz
LoggedSignal.Par.lamda = (3 * 10^9) / (LoggedSignal.Par.carier_frequence * 10^9);
LoggedSignal.Par.M = 64;
LoggedSignal.Par.total_time_step = 500;
LoggedSignal.Par.winSize = 100;
LoggedSignal.Par.dx = LoggedSignal.Par.lamda / 10;
LoggedSignal.Par.reward_mat = zeros(LoggedSignal.Par.total_time_step,1);
LoggedSignal.Par.user_max_speed = 40;
LoggedSignal.Par.ris_speed = [50 50 50];
LoggedSignal.Par.time_step = 0.01;
        
%position
LoggedSignal.Pos.bs_position = [0, 0, 25];
LoggedSignal.Pos.ris_position = [0, 0, 400];

LoggedSignal.Par.alp = 0.99;

LoggedSignal.Par.K = 7;
LoggedSignal.Mat.winSchedulMat = ones(LoggedSignal.Par.winSize,LoggedSignal.Par.K);
LoggedSignal.Mat.Test_winSchedulMat = ones(LoggedSignal.Par.winSize+1,LoggedSignal.Par.K);
LoggedSignal.Mat.Log_winSchedulMat = zeros(LoggedSignal.Par.total_time_step,LoggedSignal.Par.K);
LoggedSignal.Mat.snr_mat = zeros(LoggedSignal.Par.total_time_step,LoggedSignal.Par.K);
LoggedSignal.idx = 1;
LoggedSignal.Mat.user_movement = zeros(LoggedSignal.Par.total_time_step,3,LoggedSignal.Par.K);

[init_user_point,matric] = inituser_F(LoggedSignal.Par.user_max_speed);
LoggedSignal.Mat.user_movement(LoggedSignal.idx,:,:) = init_user_point';
LoggedSignal.move.matric = matric;
            
% Return initial environment state variables as logged signals.

InitialObservation = [zeros([1 14])];

LoggedSignal.State = InitialObservation;
InitialObservation = LoggedSignal.State';






%%
    function [init_user_point,matric] = inituser_F(max_speed)
        area_radis = 250;
        area_altit = 400;
        u_irs_rad = sqrt(area_altit^2-390^2);

        user_idx = [1,1,1,1,1,1,1]; 
        fix_s =0;
        if fix_s == 1
            for i = 1:size(user_idx,2)
                group(i,1) = i*360/7;    %degree
                group(i,2) = 25;  %speed     
                starting_point(i,1:3) = [i*360/7, u_irs_rad + i*10, i*50];
            end            
        else
            for i = 1:size(user_idx,2)
                group(i,1) = randi(360);    %degree
                group(i,2) = randi(max_speed-5)+5;  %speed     
                starting_point(i,1:3) = [randi(360), u_irs_rad + randi(area_radis-round(u_irs_rad,0)), 100];
            end
        end
        starting_point(:,1:2) = [starting_point(:,2).*cos(deg2rad(starting_point(:,1))),...
                                 starting_point(:,2).*sin(deg2rad(starting_point(:,1)))];
        init_user_point = starting_point;
        matric = [user_idx', group];
        
      
    end






end



