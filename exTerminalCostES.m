clc; close all; clear all

dt = 0.01;

%% Dynamical System

A = [2,5;
    0,2];

B = [0;2];

sys = CtSystem('StateEquation',@(t,x,u) A*x+B*u,'nx',2,'nu',1);

sys.initialCondition=[10;10];

%% Stage Cost

l = @(t,x,u) 0.1*x'*x + 10*u'*u  + (u'*u)^2;

%% Terminal Cost

stabilizingTerminalCost = 1; % <<< CHANGE HERE (0,1)

if stabilizingTerminalCost
    
    % Compute a stabilizing linear controller using LQR
    Q = eye(2); R = 1;
    [K,P] = lqr(A,B,Q,R);
    
    % For the closed-loop with the LQR controller above the following holds
    %          V = x' P x,     \dot{V}= -x' (Q + K' R K) x
    % and, therefore, an ESLyapunovCertificate is the following:
    
    cert = ESLyapunovCertificate( @(t,x)x'*P*x,2,min(eigs(P)), max(eigs(P)),min(eigs(Q + K'*R*K)));
    
    % Bound on the closed-loop stage cost
    % l(t,x,Kx) =  0.1*x'*x + 10*x'K'K x  + (x'K'K x)^2
    %           <= (0.1+10*norm(K'K ))*||x||^2  +  norm(K'K )^2||x||^4
    
    m = @(t,x) terminalCostES(t,x,cert,[0,(0.1+10*norm(K'*K )),0,norm(K'*K )^2]);
    
else
    
    m = @(t,x) 0;
end

%% Continuous-time Mpc Op

mpcOp = CtMpcOp( ...
    'System'           , sys,...
    'HorizonLength'    , 0.05,...
    'StageCost'        , l,...
    'TerminalCost'     , m);

%% Choose the Mpc Op Solver

if exist('BEGIN_ACADO') %% Use ACADO is it is installed ...
    
    mpcOpSolver = AcadoMpcOpSolver('StepSize',dt,'MpcOp',mpcOp,'AcadoOptimizationAlgorithmOptions', {...
        'KKT_TOLERANCE',1e-4,'MAX_NUM_ITERATIONS',100 ...
        });
    mpcOpSolOps = {};
    
else % ... otherwise use fmincon
    
    dtMpcOp     = DtMpcOp(mpcOp, dt);
    mpcOpSolver = FminconMpcOpSolver('MpcOp',dtMpcOp);
    mpcOp       = dtMpcOp;
    mpcOpSolOps = {'SolverOptions',optimset('Algorithm','sqp','MaxIter',10)};
    
end

sys.controller = MpcController(...
    'MpcOp'                 , mpcOp,...
    'MpcOpSolverParameters' , mpcOpSolOps, ...
    'MpcOpSolver'           , mpcOpSolver...
    );

a = VirtualArena(sys,...
    'StoppingCriteria'   , @(t,agentsList)t>1,...
    'DiscretizationStep' , dt);

a.run();

