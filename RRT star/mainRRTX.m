clc;clear;close all;
% Experimental result parameters
% Ll = Path length; Tt = Time; Node = Number of nodes
% All_Iterations = Iterations
Ll = []; Tt = []; Node = []; Cut_L=[]; All_Iterations = []; 
for j=1:1:2      % Number of runs
% Setting parameters
p=0.4;                   % Target bias probability
start_pose =[0,0];       % Starting point
global goal_pose;
goal_pose = [999,999];   % Target point
global step;
step = 20;               
r = 2*step;              
bias = step/4;           % Allowable error
numNodes = 5000;         
counter = 0;
global x_max;global y_max;
x_max = 1000;
y_max = 1000;

% Obstacle
global obstacle1;global obstacle2;global obstacle3;
obstacle1 = [550,150,200,200];     
obstacle2 = [700,650,200,200];
obstacle3 = [200,200,200,300];
or = 10;                                 % Obstacle influence range
obstacle11 = [obstacle1(1)-or,obstacle1(2)-or,obstacle1(3)+2*or,obstacle1(4)+2*or];
obstacle22 = [obstacle2(1)-or,obstacle2(2)-or,obstacle2(3)+2*or,obstacle2(4)+2*or];
obstacle33 = [obstacle3(1)-or,obstacle3(2)-or,obstacle3(3)+2*or,obstacle3(4)+2*or];
global path_V;
path_V= ([start_pose,-1]);       
path_E = [];                     % Branches of a random tree
path=[];                         % Final path

% Build map
figure(j)
axis([0 x_max 0 y_max])
rectangle('Position',obstacle1,'FaceColor',[0 .5 .5])
rectangle('Position',obstacle2,'FaceColor',[0 .5 .5])
rectangle('Position',obstacle3,'FaceColor',[0 .5 .5])
hold on
plot(start_pose,'m*','LineWidth',10);
hold on
plot(goal_pose(1),goal_pose(2),'o','LineWidth',10);
hold on

% Algorithm implementation
tic
rrts=rrt;
for i = 1:1:numNodes
    if rand(1)<=p            
        q_rand = goal_pose;
    else
        q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)];
    end
    [nearest_point,I]=rrts.nearestpoint(q_rand);            % Find the nearest node of the first tree
    new_point = rrts.newpoint(nearest_point,q_rand);        % Find a new node
    if rrts.noCollision(new_point, nearest_point, obstacle11)&&rrts.noCollision(new_point, nearest_point, obstacle22)&&rrts.noCollision(new_point, nearest_point, obstacle33)
         length = 0;                                        % The path length from the current path point to the initial point
         path_tem= [nearest_point,I]; 
          while(path_tem(1,3)>0)   
                length = length+ norm(path_tem(1,1:2)-path_V(path_tem(1,3),1:2));       % Calculate the distance from the initial node to the current node
                path_tem=path_V(path_tem(1,3),1:3);
            end
            length = length + norm(new_point-nearest_point);        % Add the distance of the new node to the nearest node
            min = length;            
            for j = 1: size(path_V,1)                               % Node judgment
                length2=0;
                if rrts.angel_dist(path_V(j,1:2),new_point)<=r      
                    path_tem = path_V(j,1:3);                       % Temporary current node
                    while(path_tem(1,3)>0)   
                        length2 = length2+ norm(path_tem(1,1:2)-path_V(path_tem(1,3),1:2));     % Calculate the distance from the initial node to the current node
                        path_tem=path_V(path_tem(1,3),1:3);  
                    end
                    length2 = length2 + norm(new_point-path_V(j,1:2));        % The distance between the new node and the nearest node
                    if length2<min
                        min = length2;    
                        nearest_point=path_V(j,1:2);
                        I=j;
                    end
                end
               
            end          
         line([nearest_point(1),new_point(1)],[nearest_point(2),new_point(2)],'color','b','LineWidth', 2);   % Draw the expansion process
         counter=counter+1;M(counter)=getframe;
         path_V = [path_V;[new_point,I]];     % Add new nodes to the random tree
    end
    % Determine whether to reach the target point
    if (norm((new_point-goal_pose),2)<=bias)
        break;
    end      
end
Iterations = i;
t=toc;              % Calculate the time consumed by a search algorithm

pose1 = path_V(size(path_V,1),1:3);
while (pose1(1,3) > 0)
    path=[pose1(1,1:2);path];
    pose1 = path_V(pose1(1,3),1:3);   
end
path = [path_V(1,1:2);path];    % Join the initial point
% Draw the final path
plot(path(:,1)',path(:,2)','color','r','LineWidth',1.5);

% Calculate the original path length
L=0;
for i=1:(size(path,1)-1)
    L=L+norm(path(i,1:2)-path(i+1,1:2),2);
end
Ll = [Ll,L];                     
Tt = [Tt,t];                
Node = [Node,size(path,1)];  
All_Iterations = [All_Iterations,Iterations];    
end
disp('Average original path length');  mean(Ll)
disp('Average experiment time');  mean(Tt)
disp('Average number of experimental path nodes');  mean(Node)
disp('Average number of experiment iterations');  mean(All_Iterations)








