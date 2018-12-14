%% MILP Formulation of Multi-Robot Task Planning with Space-Time Constraints
%
%
% $Author Dharini Dutia     `        $Created October 2018
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialization
% Cleanup
clc; clear all; close all;

% Mersenne-Twister with seed 0
% This makes the current execution repeatable
rng('default');


% Defining the environment
% Number of robots
ROBOTS = 2;   %input('Number of robots = ');

% Number of Targets/ Tasks
TASKS = 5;    %input('Enter number of tasks = ');

% Total nodes
NODES = TASKS + 1;

% Starting position of robots
start_rk = zeros(1,2); % Could be different for each robot

% Assumed Constant velocity
velocity = 10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Task Attributes
% Position of Tasks
xt = randi(10,TASKS,1);
yt = randi(10,TASKS,1);
pk = [xt,yt];

% Time required to carry out each task
dk = [0, 10,10,10,10,8]'; 
%dk = [0 1 1 1 1 1]';  
%dk = randi(50,TASKS,1);


% Start-end Time the tasks are active
ak = [0,0; 0,150; 35,150; 20,80; 50,180; 90,200];
%ak = [0,0; 0,50; 35,50; 20,60; 50,80; 90,100];
%ak = [0,0; 0,2; 4,6; 4,6; 7,9; 7,9];
Tstart = ak(:,1); 
Tend = ak(:,2); 

% Activation for tasks
fk = ones(NODES,1) ./ (Tend-Tstart);

% Number of robots required for each tasks
qk = [0, 2,1,1,2,1]'; 
%qk = [0 1 1 2 1 2 ]'; 
%qk = randi(R,TASKS,1);

%Plotting task times to check feasibility
figure(1);
for task_id = 2:NODES
    hold on;
    plot([Tstart(task_id) Tend(task_id)],task_id*ones(1,2), 'Linewidth',2);
end
title("Task Activation Window");
xlabel("time");
ylabel("Tasks");
xlim([min(Tstart),max(Tend)+1]);
ylim([0,NODES+1]);
grid on
hold off;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Graph Generation
% Node generation
nodes = [start_rk ; pk];

% Shortest distance between the nodes
e_dist = pdist2(nodes,nodes);

% Initial adjacency matrix
A = ones(NODES) - diag(ones(NODES,1));

% Rechability matrix
time_taken = e_dist/velocity;
% To check if reachable or not
g = time_taken;
for i = 1:NODES
    for j = 1:NODES
        if j==0
            g(i,j) = 0;  %can't go back to start
        end
        
        if i>1 && j>1
            if (Tend(i)+time_taken(i,j)) > (Tend(j) - dk(j))
                g(i,j) = 0;
            end
            if (Tstart(i-1)+time_taken(i,j)) > (Tend(j) - dk(j))
                g(i,j) = 0;
            end
            if (Tend(i)+time_taken(i,j)) < Tstart(j)
                g(i,j) = 0;
            end
        end
    end
end
% reach = [];
% for node =1:total_nodes
%    reach = [reach, g(node,node+1:end)];
% end
%
% Plotting the environment
% figure(2);
% G = graph(A);
% G.Edges.Weight = reach';
% h = plot(G,'XData', nodes(:,1), 'YData', nodes(:,2),'EdgeLabel',G.Edges.Weight);
% axis square
% hold on
% title("Environment");
% highlight(h,[2:1:TASKS+1])
% highlight(h,1,'NodeColor','r')

% filename = 'Environment';
% saveas(h, filename,'jpeg')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Decision variables
%
% SYMBOL : DESCRIPTION [MULTIPLICITY]
%
% xr_time(i,j)  : real-valued   for r in R, i,j in NODES [NODES * NODES * R]
% xr_ind(i,j)   : {0,1}         for r in R, i,j in NODES [NODES * NODES * R]

% Total number of variables
total_vars = NODES * NODES * ROBOTS *2;

% Obtaining ctype: for CPLEX MILP solver
% I = Integer; C = Continous; B = Binary
ctype = [ repmat('C',1,NODES*NODES*ROBOTS) repmat('B',1,NODES*NODES*ROBOTS)];


% Objective function to be minimized
f = -[repmat(fk', 1, ROBOTS*NODES), zeros(1,NODES*NODES*ROBOTS)]';


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Other constants
Q2 = min(dk(dk>0));
Q3 = max(Tend - Tstart);

ind_0 = NODES * NODES * ROBOTS;
H = min(g(g > 0));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Bounds
lb1 = zeros(NODES * NODES * ROBOTS,1);
ub1 = Q3*ones(NODES * NODES * ROBOTS,1);

lb2 = zeros(NODES * NODES * ROBOTS,1);
ub2 = ones(NODES * NODES * ROBOTS,1);

lb = [lb1;lb2];
ub = [ub1;ub2];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Quota constraints
Aineq_q2  = zeros(1,total_vars);
bineq_q2 = zeros(NODES * NODES * ROBOTS,1); %(1-Q2)*ones
row = 1;
for r=1:ROBOTS
    for i=1:NODES
        for j=1:NODES
            Aineq_q2(row, (r-1)*NODES^2 + (i-1)*NODES + j) = 1;
            Aineq_q2(row, ind_0 + (r-1)*NODES^2 + (i-1)*NODES + j) = -Q2;
            row = row+1;
        end
    end
end

Aineq_q3  = zeros(1,total_vars);
bineq_q3 = zeros(NODES * NODES * ROBOTS,1);
row = 1;
for r=1:ROBOTS
    for i=1:NODES
        for j=1:NODES
            Aineq_q3(row, (r-1)*NODES^2 + (i-1)*NODES + j) = 1;
            Aineq_q3(row, ind_0 + (r-1)*NODES^2 + (i-1)*NODES + j) = -Q3;
            row = row+1;
        end
    end
end

Aeq_q4  = zeros(1,total_vars);
beq_q4 = qk;
for r=1:ROBOTS
    for i=1:NODES
        for j=1:NODES
            Aeq_q4(j, ind_0 + (r-1)*NODES^2 + (i-1)*NODES + j) = 1;
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Correctness constraints

Aineq_c1  = zeros(1,total_vars);
bineq_c1 = ones(NODES * ROBOTS,1);
for r=1:ROBOTS
    for i=1:NODES
        for j=1:NODES
            Aineq_c1((r-1)*NODES + i, ind_0 + (r-1)*NODES^2 + (i-1)*NODES + j) = 1;
        end
    end
end

Aineq_c2  = zeros(1,total_vars);
bineq_c2 = ones(NODES * ROBOTS,1);
for r=1:ROBOTS
    for i=1:NODES
        for j=1:NODES
            Aineq_c2((r-1)*NODES + j, ind_0 + (r-1)*NODES^2 + (i-1)*NODES + j) = 1;
        end
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Reachability
AineqH  = zeros(1,total_vars);
bineqH = zeros(NODES * ROBOTS,1);
row = 1;
for r=1:ROBOTS
    for i=1:NODES
        for j=1:NODES
            AineqH(row, ind_0 + (r-1)*NODES^2 + (i-1)*NODES + j) = H;
            bineqH(row,1) = g(i,j);
            row = row+1;
        end
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Motion constraints
% Make diagonal matrix of constants
M = zeros(NODES,NODES);
for j = 1:NODES
    for k = 1:NODES
        if j ~= k
            M(j,k) = Tend(j) + dk(j) + time_taken(j,k) - Tend(k);
        end
    end
end
Mdiag = diag(reshape(M', [], 1));

% Make matrix of coefficients for xind
AineqMRstride = NODES*NODES;
AineqMind = [];
for i = 1:ROBOTS
    AineqMindLpad = zeros(NODES * NODES, (i-1)*AineqMRstride);
    AineqMindRpad = zeros(NODES * NODES, (ROBOTS-i)*AineqMRstride);
    AineqMind = [ AineqMind; AineqMindLpad Mdiag AineqMindRpad];
end

% Make matrix of coefficients for xtime
AineqMTstride = NODES * NODES;
AineqMtime = [];
for i = 1:ROBOTS
    for j = 1:NODES
        for k = 1:NODES
            AineqMtimerow = zeros(1,ROBOTS * AineqMRstride);
            if j ~= k
                AineqMtimerow((i-1)*NODES*NODES + (j-1)*NODES + k) = 1;
                for l = 1:NODES
                    AineqMtimerow((i-1)*NODES*NODES + (l-1)*NODES + j) = -1;
                end
            end
            AineqMtime = [ AineqMtime ; AineqMtimerow ];
        end
    end
end

% Combine the two
AineqM = [ AineqMtime AineqMind ];
bineqM = zeros(ROBOTS*NODES*NODES,1);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Flow constraints
Aeq_flow = zeros(ROBOTS*NODES,total_vars);
beq_flow = zeros(ROBOTS*NODES,1);
for r = 1:ROBOTS
    for i = 1:NODES
        row = (r-1)*NODES + i;
        for j = 1:NODES
            Aeq_flow(row,ind_0+(r-1)*NODES^2 + (i-1)*NODES + j) = 1;
            Aeq_flow(row,ind_0+(r-1)*NODES^2 + (j-1)*NODES + i) = -1;
        end
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CPLEX optimization

% Combining the matrices
Aineq = [Aineq_q2;Aineq_q3;Aineq_c1;Aineq_c2;AineqM;AineqH];
bineq = [bineq_q2;bineq_q3;bineq_c1;bineq_c2;bineqM;bineqH];
Aeq = [Aeq_q4];%Aeq_flow];
beq = [beq_q4];%beq_flow];

% To get the total number of in/equality equations
[eq_count,~] = size(beq);
[ineq_count,~] = size(bineq);

% Final formulation
tic
X = cplexmilp(f,Aineq,bineq,Aeq,beq,"",[],[],lb,ub,ctype)
toc


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Post Processing
% Sequence of visit
if ~isempty(X)
    order = {};
    temp = [];
    %start pos = node 0
    for r=1:ROBOTS
        for i=1:NODES
            for j=1:NODES
                if X(ind_0+ (r-1)*NODES^2 + (i-1)*NODES + j) == 1
                    temp = [temp,j];
                    
                end
            end
            order{i,r} = temp;
            temp = [];
        end
    end
    
    
    
    % Visualize plan for each robot
    Ibase  = ROBOTS*NODES*NODES;
    Irobot = NODES*NODES;
    schedule = [];
    for r = 1:ROBOTS
        curr_node = 1;
        Rtrips = reshape(X(Irobot * (r-1) + 1:Irobot * r), NODES, NODES)';
        Rind = reshape(X(Ibase + Irobot * (r-1) + 1:Ibase + Irobot * r), NODES, NODES)';
        for i = 1:NODES
            next_node = find(Rtrips(curr_node,:) > 1e-6);
            if ~isempty(next_node)
                arrive = Tend(next_node) - Rtrips(curr_node, next_node);
                leave = arrive + dk(next_node);
                schedule = [schedule; r curr_node next_node arrive leave qk(next_node)];
                curr_node = next_node;
            else
                curr_node = i;
            end
        end
    end
    % Arrival and departure time schedule
    schedule
end


