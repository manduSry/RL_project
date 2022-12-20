clear ; clc;

Obs_Info = rlNumericSpec([7*2 1]);
user = 1:7;
C = nchoosek(user,4);
cc = [];
for i = 1:size(C,1)
    cc{end+1} = C(i,:);
end
Act_Info = rlFiniteSetSpec(cc);
env = rlFunctionEnv(Obs_Info,Act_Info,'step_rl_1213','rest_RLp');

%env = rlFunctionEnv(Obs_Info,Act_Info,'step_RLp','rest_RLp');
clear cc; clear C; clear i; clear user;
%%
dnn = [
    featureInputLayer(Obs_Info.Dimension(1),'Normalization','none','Name','state')
    fullyConnectedLayer(64,'Name','CriticStateFC1')
    reluLayer('Name','CriticRelu1')
    fullyConnectedLayer(64, 'Name','CriticStateFC2')
    reluLayer('Name','CriticCommonRelu')
    fullyConnectedLayer(length(Act_Info.Elements),'Name','output')];
dnn = dlnetwork(dnn);

criticOpts = rlOptimizerOptions('LearnRate',0.001,'GradientThreshold',1);
critic = rlVectorQValueFunction(dnn,Obs_Info,Act_Info);


agentOpts = rlDQNAgentOptions(...
    'UseDoubleDQN',true, ...
    'TargetSmoothFactor',0.01, ...
    'TargetUpdateFrequency',4, ...   
    'ExperienceBufferLength',20000, ...
    'CriticOptimizerOptions',criticOpts, ...
    'MiniBatchSize',32);
agentOpts.EpsilonGreedyExploration.Epsilon = 0.95;
agent = rlDQNAgent(critic,agentOpts);




%%

trainOpts = rlTrainingOptions;

trainOpts.MaxEpisodes = 2700;%(20*12)*5;
trainOpts.MaxStepsPerEpisode = 500;
trainOpts.StopTrainingCriteria = "AverageReward";
trainOpts.StopTrainingValue = 500*2000;
trainOpts.ScoreAveragingWindowLength = 5;

trainOpts.SaveAgentCriteria = "EpisodeReward";
trainOpts.SaveAgentValue = 500;
trainOpts.SaveAgentDirectory = "agent";
trainOpts.Verbose = false;
trainOpts.Plots = "training-progress";
trainingInfo = train(agent,env,trainOpts);

%%
save(trainOpts.SaveAgentDirectory+ '/DDQN_agent_1215.mat','agent')
agent = load('DDPG_agent_action1.mat');

load_idx = 1;
if load_idx == 1
    agent_path = "agent/DQN_agent_f.mat";
    agent = load(agent_path);
    agent = agent.agent;
    simOptions = rlSimulationOptions(MaxSteps=500);
    experience = sim(env,agent,simOptions);
    act = squeeze(experience.Action.act1.Data);

    obs = squeeze(experience.Observation.obs1.Data);

end

%%
plotin_result = 1;
if plotin_result ==1
    figure(111)
    close all;
    user_move = env.LoggedSignals.Mat.user_movement;
    ris_move = env.LoggedSignals.Pos.ris_position;
    bs = env.LoggedSignals.Pos.bs_position;
    for i = 1:size(user_move,3)
        plot3(user_move(:,1,i),user_move(:,2,i),user_move(:,3,i),'b-')
        hold on

    end
    for i = 1:size(user_move,3)
        plot3(user_move(1,1,i),user_move(1,2,i),user_move(1,3,i),'bs')
        plot3(user_move(500,1,i),user_move(500,2,i),user_move(500,3,i),'bo')
        hold on

        plot3(ris_move(1),ris_move(2),ris_move(3),'rs')
        hold on
        grid on
    end
    %test 
    tt = sum(env.LoggedSignals.Mat.Log_winSchedulMat);
    [~,idd] = max(tt);
    plot3(user_move(1,1,idd),user_move(1,2,idd),user_move(1,3,idd),'ks','LineWidth',2)
    plot3(user_move(500,1,idd),user_move(500,2,idd),user_move(500,3,idd),'ko','LineWidth',2)
    hold on
    [~,idd] = min(tt);
    plot3(user_move(1,1,idd),user_move(1,2,idd),user_move(1,3,idd),'ms','LineWidth',2)
    plot3(user_move(500,1,idd),user_move(500,2,idd),user_move(500,3,idd),'mo','LineWidth',2)
    hold on
    %
    
    scheduling_point = squeeze(experience.Observation.obs1.Data);
    scheduling_point = scheduling_point(1:3,:)';
    plot3(scheduling_point (2:end,1),scheduling_point (2:end,2),scheduling_point (2:end,3),'k-')
    hold on
    
    xlim([-250 250])
    ylim([-250 250])

end

figure(666)
%DDQN_P1_Agent270
x = 1:1:269;
y_idx = ['2_DDQN_P1_Agent270';'2_DDQN_P2_Agent270';'2_DDQN_P3_Agent270';'2_NDQN_P1_Agent270'];

for i = 1:4
    y = load(strcat('/home/cnlab1/바탕화면/JS/IRS_RL/agent/',y_idx(i,:),'.mat'));
    plot(x,y.savedAgentResult.AverageReward(x),'-','LineWidth',2)
    hold on;
end
grid on;
legend('DDQN P1','DDQN P2','DDQN P3','DQN P1');
xlim([1,269])
ylim([4400, 5000])
xlabel('Eposide')
ylabel('Eposide reward')








