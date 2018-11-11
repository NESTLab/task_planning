%% MILP Formulation of Multi-Robot Task Allocation with Space-Time Constraints
%
%
% $Author Dharini Dutia     `        $Created October 2018
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%% Initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cleanup
clc; clear all; close all;
% Mersenne-Twister with seed 0
% This makes the current execution repeatable
rng('default');


% Defining the environment
% Number of robots
R = 2;   %input('Number of robots = ');
% Starting position of robots
start_rk = zeros(R,2); % Could be different for each robot


%%%%%%%%%%%%%%%%%%%%%%%%%% Task Attributes %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Number of Targets/ Tasks
tau = 5;    %input('Enter number of tasks = '); 
% Position of Tasks
xt = randi(10,tau,1);
yt = randi(10,tau,1); 
pk = [xt,yt];
% Start-end Time the tasks are active
ak_start = randi(50,tau,1);
%ak = [ak_start, ak_start+(1/fk)];
ak = [0,100 ; 5,150; 20,300; 10,400; 35,500];
% Activation for tasks
fk = (ak(:,2) - ak(:,1)).^(-1);

%Plotting task times to check feasibility
figure(1);
for task_id = 1:tau
    hold on;
    plot(ak(task_id,:),task_id*ones(1,2), 'Linewidth',2);
end
title("Task activation graph");
xlabel("time");
ylabel("Tasks");
xlim([min(ak(:,1))-1,max(ak(:,2))+1]);
ylim([0,tau+1]);
hold off;

% Number of robots required for each tasks
qk = ones(tau,1); %randi(R,tau,1);

% Time required to carry out each task
dk = randi(50,tau,1);


%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graph Generation %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Node generation
nodes = [start_rk(1,:) ; pk];
[total_nodes,~] = size(nodes);

% Shortest distance between the nodes
e_dist = pdist2(nodes,nodes);

% Assumed Constant velocity
velocity = 10;

% Initial adjacency matrix
A = ones(total_nodes) - diag(ones(total_nodes,1));

% Rechability matrix
time_taken = e_dist/velocity;
% To check if reachable or not
g = time_taken;
for i = 1:total_nodes
    for j = 1:total_nodes
        %        if (ak(i,2)+time_taken(i,j)) < (ak(j,1) + dk)
        %            g(i,j) = 0;
        %        end
        if (i>1) && (j>1)
            if (ak(i-1,1)+time_taken(i,j)) > (ak(j-1,2) - dk)
                g(i,j) = 0;
            end
            if (ak(i-1,2)+time_taken(i,j)) < ak(j-1,1)
                g(i,j) = 0;
            end
        else
            if (j==1)   % Don't allow to go back to start node
                g(i,j) = 0;
            end
        end
    end
end
reach = [];
for node =1:total_nodes
   temp = g(node,:);
   temp(node) = [];
   reach = [reach, temp];
end

% Plotting the environment
figure(2);
G = digraph(A);
G.Edges.Weight = reach';
h = plot(G,'XData', nodes(:,1), 'YData', nodes(:,2),'EdgeLabel',G.Edges.Weight);
axis square
hold on
title("Environment");
highlight(h,[2:1:tau+1])
highlight(h,1,'NodeColor','r')

% filename = 'Environment';
% saveas(h, filename,'jpeg')

%close all;

%%%%%%%%%%%%%%%%%%%%%%%%%% Decision variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% SYMBOL : DESCRIPTION [MULTIPLICITY]
%
% xr_time(i,j)  : real-valued   for r in R, i,j in nodes [total_nodes * total_nodes * R]
% xr_ind(i,j)   : {0,1}         for r in R, i,j in nodes [total_nodes * total_nodes * R]

% Total number of variables
total_vars = total_nodes * total_nodes * R *2;

% Obtaining ctype: for CPLEX MILP solver
% I = Integer; C = Continous; B = Binary
ctype = [];
for i =1:total_vars/2
    ctype = [ctype; char('I')];
end
for i =total_vars/2 +1 :total_vars
    ctype = [ctype; char('B')];
end

% Objective function to be minimized  %TODO here maximize
f = [];
const = [1;fk]; %adding start node
for r=1:R
    for i=1:total_nodes
        for j=1:total_nodes
            f = [f,-const(i,1)];
        end
    end
end
f = [f,zeros(1,total_nodes * total_nodes * R)];


%%%%%%%%%%%%%%%%%%%%%%%% Activation constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%
lb1 = [];
const = [0;dk];  %adding start node
for r=1:R
    for i=1:total_nodes
        for j=1:total_nodes
            lb1 = [lb1;const(j)];
        end
    end
end
ub1 = Inf*ones(total_nodes * total_nodes * R,1); 

lb2 = zeros(total_nodes * total_nodes * R,1);
ub2 = ones(total_nodes * total_nodes * R,1); 

lb = [lb1;lb2];
ub = [ub1;ub2];

%%%%%%%%%%%%%%%%%%%%%%%%%%% Quota constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q2 = min(dk);
Q3 = max(ak(:,2) -ak(:,1)) + 1;
ind_0 = total_nodes * total_nodes * R;

Aineq_q2  = zeros(1,total_vars);
bineq_q2 = (1-Q2)*ones(total_nodes * total_nodes * R,1);
row = 1;
for r=1:R
    for i=1:total_nodes
        for j=1:total_nodes
            Aineq_q2(row, (r-1)*total_nodes^2 + (i-1)*total_nodes + j) = -1;
            Aineq_q2(row, ind_0 + (r-1)*total_nodes^2 + (i-1)*total_nodes + j) = 1;
            row = row+1;
        end
    end 
end

Aineq_q3  = zeros(1,total_vars);
bineq_q3 = zeros(total_nodes * total_nodes * R,1);
row = 1;
for r=1:R
    for i=1:total_nodes
        for j=1:total_nodes
            Aineq_q3(row, (r-1)*total_nodes^2 + (i-1)*total_nodes + j) = 1;
            Aineq_q3(row, ind_0 + (r-1)*total_nodes^2 + (i-1)*total_nodes + j) = -Q3;
            row = row+1;
        end
    end
end

Aeq_q4  = zeros(1,total_vars);
beq_q4 = [0;qk];     %adding start node
for r=1:R
    for i=1:total_nodes
        for j=1:total_nodes
            Aeq_q4(j, ind_0 + (r-1)*total_nodes^2 + (i-1)*total_nodes + j) = 1;
        end
    end
end


%%%%%%%%%%%%%%%%%%%%%%%% Correctness constraints %%%%%%%%%%%%%%%%%%%%%%%%%%

Aineq_c1  = zeros(1,total_vars);
bineq_c1 = ones(total_nodes * R,1);
for r=1:R
    for i=1:total_nodes
        for j=2:total_nodes
            Aineq_c1((r-1)*total_nodes + i, ind_0 + (r-1)*total_nodes^2 + (i-1)*total_nodes + j) = 1;
        end
    end
end

Aineq_c2  = zeros(1,total_vars);
bineq_c2 = ones(total_nodes * R,1);
for r=1:R
    for i=1:total_nodes
        for j=2:total_nodes
            Aineq_c2((r-1)*total_nodes + j, ind_0 + (r-1)*total_nodes^2 + (i-1)*total_nodes + j) = 1;
        end
    end
end


%%%%%%%%%%%%%%%%%%%%%%%% Motion constraints %%%%%%%%%%%%%%%%%%%%%%%%%%
H = min(reach(reach>0));

Aineq_m  = zeros(1,total_vars);
bineq_m = zeros(total_nodes * R,1);
row = 1;
for r=1:R
    for i=1:total_nodes
        for j=1:total_nodes
            Aineq_m(row, ind_0 + (r-1)*total_nodes^2 + (i-1)*total_nodes + j) = H;
            bineq_m(row,1) = g(i,j);
            row = row+1;
        end
    end
end



%% CPLEX optimization

% Combining the matrices
Aineq = [Aineq_q2;Aineq_q3;Aineq_c1;Aineq_c2;Aineq_m];
bineq = [bineq_q2;bineq_q3;bineq_c1;bineq_c2;bineq_m];
Aeq = [Aeq_q4];
beq = [beq_q4];

% To get the total number of in/equality equations
[eq_count,~] = size(beq);
[ineq_count,~] = size(bineq);

%%%%%%%%%%%%%%%%%%%%%%%%%%% Final formulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic
X = cplexmiqp([],f,Aineq,bineq,Aeq,beq,[],[],[],lb,ub,ctype')

%x = cplexmilp(f,Aineq,bineq,Aeq,beq,sostype,sosind,soswt,lb,ub,ctype)
%x = cplexmiqp(H,f,Aineq,bineq,Aeq,beq,sostype,sosind,soswt,lb,ub,ctype)
toc


