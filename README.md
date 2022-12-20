# RL_project

본 과제는 MATLAB 코드로 작성됨

source code 폴더 안의 main_RLp.m 파일이 메인 코드

11번째 줄 
  env = rlFunctionEnv(Obs_Info,Act_Info,'step_rl_1213','rest_RLp');
에서 커스텀 환경을 생성함

16~26번째 줄
critic 네트워크의 neural network 구조와 하이퍼 파라미터

29~37번째 줄
agent의 하이퍼 파라미터

44~57번째 줄
train 파라미터

60~74번째 줄
train 하면서 agent를 save 새로운 agent 폴더가 생기며 폴더 안에 agent가 저장

**중요
77번째 줄
plotin_result = 0;
으로 설정 해야함
**

77~end번째 줄
하이퍼파라미터 및 agent를 변화시켜 eposide당 reward 그래프 출력 
