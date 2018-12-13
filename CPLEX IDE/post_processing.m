%% Post-processing of Optimal solution
% $Author Dharini Dutia     `        $Created November 2018
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%% Initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear all; close all;

sheetname = 'my_sheet.xls';
%sheetname = 'oldHmin.xls';
%sheetname = 'oldHmax.xls';
%sheetname = 'oldHmax_signwrong.xls';
%sheetname = 'newHmin.xls';
%sheetname = 'newHmax.xls';

% Reading the solution generated by CPLEX IDE
xind = xlsread(sheetname, 'A1:A72');
xtime = xlsread(sheetname, 'B1:B72');


% other constants
TASKS = xlsread(sheetname, 'C1:C1');
ROBOTS = xlsread(sheetname, 'C2:C2');

NODES = TASKS + 1;
R_vars = NODES^2;
total_vars = NODES^2 *ROBOTS;

dk = xlsread(sheetname, 'D1:D6');
qk = xlsread(sheetname, 'D7:D12');
task_start = xlsread(sheetname, 'E1:E6');
task_end = xlsread(sheetname, 'F1:F6');
time_taken = xlsread(sheetname, 'G1:L6');
g = xlsread(sheetname, 'M1:R6');

%%%%%%%%%%%%%%%%%%%%%%%%%%%% sequence of visit %%%%%%%%%%%%%%%%%%%%%%%%
order = {};
temp = [];
%start pos = node 0
for r=1:ROBOTS
    for i=1:NODES
        for j=1:NODES
            if xind((r-1)*R_vars + (i-1)*NODES + j) == 1
                    temp = [temp,j];
                
            end
        end
        order{i,r} = temp;
        temp = [];
    end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Visualize plan for each robot
Ibase  = ROBOTS*NODES*NODES;
Irobot = NODES*NODES;
schedule = [];
for r = 1:ROBOTS
    curr_node = 1;
    leave = 0;
    
    Rtrips = reshape(xtime(Irobot * (r-1) + 1:Irobot * r), NODES, NODES)';
    Rind = reshape(xind(Irobot * (r-1) + 1:Irobot * r), NODES, NODES)';
    for i = 1:NODES
        next_node = find(Rtrips(curr_node,:) > 1e-6);
        if ~isempty(next_node)
            task_end(next_node-1)
            Rtrips(next_node, curr_node)
            arrive = task_end(next_node-1) - Rtrips(curr_node, next_node);
            leave = arrive + dk(next_node);
            schedule = [schedule; r curr_node next_node arrive leave qk(next_node)];
            curr_node = next_node;
        else
            curr_node = i;
        end
    end
end

schedule
