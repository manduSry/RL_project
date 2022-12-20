function [NextObs,Reward,IsDone,LoggedSignal] = step_rl_1213(Action,LoggedSignal)
% Custom step function to construct cart-pole environment for the function
% name case.
%
% This function applies the given action to the environment and evaluates
% the system dynamics for one simulation step.

% Define the environment constants.

LoggedSignal.idx = LoggedSignal.idx+1;


%User movement
this_user_p = squeeze(LoggedSignal.Mat.user_movement(LoggedSignal.idx-1,:,:))';
[next_user_position] = usermove_next(this_user_p,LoggedSignal.move.matric,LoggedSignal.Par.time_step);

%User area constraint
[matric] = cutting(next_user_position,LoggedSignal.move.matric);

LoggedSignal.move.matric = matric;
LoggedSignal.Mat.user_movement(LoggedSignal.idx,:,:) = next_user_position';
g_idx = 0:LoggedSignal.Par.N(1) * LoggedSignal.Par.N(2) / LoggedSignal.Par.risWin -1;
g_phase =  exp(-1j*2*pi .* g_idx * LoggedSignal.Par.dx/LoggedSignal.Par.lamda * (LoggedSignal.Pos.ris_position(1)-LoggedSignal.Pos.bs_position(1))...
    /norm(LoggedSignal.Pos.ris_position(1:3)-LoggedSignal.Pos.bs_position(1:3)));
G = exp(-1j*2*pi*norm(LoggedSignal.Pos.ris_position(1:3)-LoggedSignal.Pos.bs_position(1:3))/LoggedSignal.Par.lamda)*...
    g_phase;
[snr]= scheduling(LoggedSignal,G,Action);
LoggedSignal.Mat.snr_mat(LoggedSignal.idx,Action) = snr;

%[ideal_snr] = idealSnr(LoggedSignal,ind);
% [reward_snr]= reward_Snr(LoggedSignal,ind);

% fprintf('cal snr\n')
% disp(snr)
% fprintf('ideal P snr\n')
% disp(ideal_snr)


%test count
idx_sch = zeros([1 7]);
idx_sch(Action(:))  = 1;
LoggedSignal.Mat.Test_winSchedulMat(end,:) = idx_sch;
LoggedSignal.Mat.Log_winSchedulMat(LoggedSignal.idx,:) = idx_sch;
LoggedSignal.Mat.winSchedulMat(1:end,:) = LoggedSignal.Mat.Test_winSchedulMat(2:end,:);

LoggedSignal.Mat.Test_winSchedulMat(1:(end-1),:) = LoggedSignal.Mat.winSchedulMat(1:end,:);

%reward
rreward =  reward_F(snr,LoggedSignal.Mat.winSchedulMat);
Reward = rreward;
%ideal_rreward =  reward_F(reward_snr,power*LoggedSignal.Par.time_step);
LoggedSignal.Par.reward_mat(LoggedSignal.idx) = rreward;

r = zeros([1, 7]);
for i1 =1:size(LoggedSignal.Mat.winSchedulMat,2)
    f_idx = find(LoggedSignal.Mat.winSchedulMat(:,i1)==1,1,'last'); 
    if size(f_idx,1) == 0 
        r(i1) = 0;
    else
        r(i1) = f_idx;
    end
end
        
LoggedSignals.State = squeeze(LoggedSignal.Mat.snr_mat(LoggedSignal.idx,:));
LoggedSignals.State  = [LoggedSignals.State r];

NextObs = LoggedSignals.State';



if LoggedSignal.idx == 500
%     re = LoggedSignal.Par.reward_mat(:);
%     save_path = 'C:\Users\B2\Desktop\lee\2022\1.CNL\4.project\9.aoi\code\1106\rward\';
%     save('reward.mat','re')
    IsDone = 1;
else
    IsDone = 0;
end

if rem(LoggedSignal.idx,100) == 0 
    fprintf('idx\n')
    disp(LoggedSignal.idx)
    fprintf('Obs\n')
    disp(round(NextObs(1:7),2))
    disp(round(NextObs(8:14)',2))


    %disp(new_ris_move)
    fprintf('Reward\n')
    disp(round(Reward,2))    
end



%%

    function [matric] = cutting(next_movement_tmp,matric)
        for i = 1:size(next_movement_tmp,1)
           ttmp_move(i) = norm(next_movement_tmp(i,1:2));
        end
        
        ttmp_move(ttmp_move >=225) = 1;
        ttmp_move(ttmp_move <225 &ttmp_move >1) = -1;
        for i = 1:size(ttmp_move,2)
            if ttmp_move(i) == 1
                matric(i,2) = 0.1*randn(1)+180+matric(1,2);
            end
        end

    end
    
    function [next_user_position] = usermove_next(this_user_p,matric,time_step)
        alp = 0.5;
        tmp_move = zeros(size(this_user_p,1),3);

        tmp_move(:,1:2) = ( alp*matric(:,3) + (1-alp)*0.5*randn([7,1]) ) .*...
             alp.*[...
             cos( matric(:,1) + (1-alp)*randn([7,1]) ), sin( matric(:,2) + (1-alp)*randn([7,1]) ) ...
             ] ;
 
        next_user_position = this_user_p + time_step*tmp_move;

        
    end

    function [snr] = scheduling(LoggedSignal,G,Action)
        ind=Action;
        snr = ind;
        h = zeros(LoggedSignal.Par.risWin, LoggedSignal.Par.N(1) * LoggedSignal.Par.N(2) / LoggedSignal.Par.risWin);
        tmp_s = 0:LoggedSignal.Par.N(1)* LoggedSignal.Par.N(2) / LoggedSignal.Par.risWin-1;
        optimal_phase_mat = zeros(size(h,1),size(h,2),size(h,2));
%         g_phase =  exp(-1j*2*pi .* g_idx * LoggedSignal.Par.dx/LoggedSignal.Par.lamda * (LoggedSignal.Pos.ris_position(1)-LoggedSignal.Pos.bs_position(1))...
%     /norm(LoggedSignal.Pos.ris_position(1:3)-LoggedSignal.Pos.bs_position(1:3)));
% G = sqrt(10^(LoggedSignal.Par.beta_0/10)/norm(LoggedSignal.Pos.ris_position(1:3))^2)*exp(-1j*1*pi)*g_phase;
        for i = 1:size(h,1)
                h_phase = exp(-1j*2*pi.*tmp_s* LoggedSignal.Par.dx/LoggedSignal.Par.lamda * (LoggedSignal.Pos.ris_position(1)-LoggedSignal.Mat.user_movement(LoggedSignal.idx,1,ind(i)))...
                             /norm(LoggedSignal.Pos.ris_position(1:3)-LoggedSignal.Mat.user_movement(LoggedSignal.idx,1:3,ind(i))));
                h(i,:) = exp(-1j*2*pi*norm(LoggedSignal.Pos.ris_position(1:3)-LoggedSignal.Mat.user_movement(LoggedSignal.idx,1:3,ind(i)))/LoggedSignal.Par.lamda)*...
                         h_phase;   %[1 N]
                optimal_phase = exp( 1j*( -angle(h(i,:)) ) ); %[N 1]
                %optimal_phase = exp( 1j*( -angle(h(i,:)) ) ).*exp( 1j*( -angle(G) ) ); %[N 1]

                optimal_phase_mat(i,:,:) = diag(optimal_phase);
        end
        ris_ampNoiseL = 0;
        amp = 1;
        tmp_sch = zeros(1,LoggedSignal.Par.K);

        for iii = 1:LoggedSignal.Par.risWin
            test_h = h;
            test_h(iii,:) = [];
            infer = test_h * squeeze(optimal_phase_mat(iii,:,:)) * ones([LoggedSignal.Par.N(1)* LoggedSignal.Par.N(2) / LoggedSignal.Par.risWin 1]);
            infer_snr = sum(abs(infer));
%             snr2 = 10^((LoggedSignal.Par.P - 30) / 10) * (10^ (LoggedSignal.Par.beta_0/10))^2 ... 
%                          * 1 / (LoggedSignal.Par.risWin)* infer_snr ...
%                        *1/(...
%                        norm(LoggedSignal.Pos.ris_position(1:3)-LoggedSignal.Pos.bs_position(1:3))^2 * norm(...
%                            LoggedSignal.Pos.ris_position(1:3) - LoggedSignal.Mat.user_movement(LoggedSignal.idx,1:3,ind(iii)))^2)...
%                        + ris_ampNoiseL + 10^((LoggedSignal.Par.sigma - 30) / 10);
%LoggedSignal.Par.N(1) * LoggedSignal.Par.N(2) / LoggedSignal.Par.risWin)^2 * amp^2 / ...
            snr1 = (...
                        norm(...
                            LoggedSignal.Pos.ris_position(1:3)-LoggedSignal.Pos.bs_position(1:3))^2 *...
                        norm(...
                            LoggedSignal.Pos.ris_position(1:3) - LoggedSignal.Mat.user_movement(LoggedSignal.idx,1:3,ind(iii)))^2 ...
                            )...
                            * 10^((LoggedSignal.Par.sigma - 30) / 10)/...
                            (10^((LoggedSignal.Par.P - 30) / 10) * (10^ (LoggedSignal.Par.beta_0/10))^2);                         
%             snr1 = 10^((LoggedSignal.Par.P - 30) / 10) * (10^ (LoggedSignal.Par.beta_0/10))^2 ... 
%                          * 1 / LoggedSignal.Par.risWin * ...
%                          abs( exp(1j*angle(h(iii,:))) * squeeze(optimal_phase_mat(iii,:,:)) * exp(1j*angle(G.')))^2 /...           
%                           (...
%                        norm(LoggedSignal.Pos.ris_position(1:3)-LoggedSignal.Pos.bs_position(1:3))^2 * norm(...
%                            LoggedSignal.Pos.ris_position(1:3) - LoggedSignal.Mat.user_movement(LoggedSignal.idx,1:3,ind(iii)))^2 ...
%                            );
            snr(iii) = 10 * log10(...
                       (LoggedSignal.Par.N(1) * LoggedSignal.Par.N(2) / LoggedSignal.Par.risWin)^2 ...
                        /(infer_snr + snr1));
      
            tmp_sch(ind(iii))  = 1;
        end




    end
            
    function reward =  reward_F(snr,winSch)
       
        discount_win = 0.99;
        snr(snr<2) = 0;
        snr = log2(1+snr);
        reward = sum(snr,2);
        r = zeros([1 size(winSch,2)]);
        
        for i =1:size(winSch,2)
            f_idx = find(winSch(:,i)==1,1,'last'); 
            if size(f_idx,1) == 0 
                r(i) = 0;
            else
                r(i) = f_idx;
            end

        end
%         disp(discount_win^(size(winSch,1)*size(winSch,2) - sum(r)))
        reward = discount_win^(size(winSch,1)*size(winSch,2) - sum(r)) * reward;
%        reward = (size(winSch,1)*size(winSch,2) - sum(r));

    end




end
            
            