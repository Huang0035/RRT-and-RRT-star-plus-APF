function rrts = rrt
rrts.isEdgeCollisionFree= @isEdgeCollisionFree;           % Determine whether there is an obstacle in the line between two points
rrts.isOutOfBounds = @isOutOfBounds;                      % Judging whether the point exceeds the obstacle
rrts.euclidian_dist = @euclidian_dist;                    % Calculate the distance between two points
rrts.arangel= @ arangel;                                  % Calculates the angle between two vectors
rrts.direction = @direction;                              % Compute a two-point vector
rrts.samplesobol = @samplesobol;                          % Sample Sobol sequence
rrts.nearestpoint = @nearestpoint;                        % Calculate nearest node
rrts.nearestpoint2 = @nearestpoint2;                      % Calculate the closest node of the second tree
rrts.angel_dist = @angel_dist;                            % Calculate the distance between joint angles
rrts.newpoint = @newpoint;                                % Generate new nodes
rrts.findparentpoint = @findparentpoint;                  % Find the parent node of the current node
rrts.ccw=@ccw;
rrts.noCollision=@noCollision;
end

function val = ccw(A,B,C)
    val = (C(2)-A(2)) * (B(1)-A(1)) > (B(2)-A(2)) * (C(1)-A(1));
end

function nc = noCollision(n2, n1, o)
    A = [n1(1) n1(2)];
    B = [n2(1) n2(2)];
    obs = [o(1) o(2) o(1)+o(3) o(2)+o(4)];   
    C1 = [obs(1),obs(2)];
    D1 = [obs(1),obs(4)];
    C2 = [obs(1),obs(2)];
    D2 = [obs(3),obs(2)];
    C3 = [obs(3),obs(4)];
    D3 = [obs(3),obs(2)];
    C4 = [obs(3),obs(4)];
    D4 = [obs(1),obs(4)];
    ints1 = ccw(A,C1,D1) ~= ccw(B,C1,D1) && ccw(A,B,C1) ~= ccw(A,B,D1); 
    ints2 = ccw(A,C2,D2) ~= ccw(B,C2,D2) && ccw(A,B,C2) ~= ccw(A,B,D2);
    ints3 = ccw(A,C3,D3) ~= ccw(B,C3,D3) && ccw(A,B,C3) ~= ccw(A,B,D3);
    ints4 = ccw(A,C4,D4) ~= ccw(B,C4,D4) && ccw(A,B,C4) ~= ccw(A,B,D4);
    if ints1==0 && ints2==0 && ints3==0 && ints4==0
        nc = 1;
    else
        nc = 0;
    end
end

% Compute the vector determined by two points
function [dir] = direction(pose1,pose2)
    dir1 = pose2(1)-pose1(1);
    dir2 = pose2(2)-pose1(2);
    dir = [dir1 dir2];   
end

% Calculate the angle between two phases
function Flag = angel_limit(path,xnear,xnew)
    global angle;
    if xnear(3)<0
        Flag = 1;
    else
        xinit = path(xnear(3),1:3);
        A = [xinit(1)-xnear(1),xinit(2)-xnear(2)];
        B = [xnew(1)-xnear(1),xnew(2)-xnear(2)];
        thea = acos(dot(A,B)/(norm(A)*norm(B)))*180/pi;
        if thea>=angle
            Flag = 1;
        else
            Flag = 0; 
        end
    end
end

% Find the distance between two points
function [angel_len] = angel_dist(pose1,pose2)
    angel_len = norm((pose2-pose1),2);
end

% Calculate nearest node
function [thea_dis,I]= nearestpoint(random_point)
    global path_V;
    min_dis = inf;
    for i = 1: size(path_V,1)
        if(min_dis>angel_dist(random_point,path_V(i,1:2)))
            min_dis = angel_dist(random_point,path_V(i,1:2));
            thea_dis = [path_V(i,1:3)];
            I = i;
        end        
    
    end
end

% Find a new node according to the step size
function [new_thea] = newpoint(near_point,random_point)
    global step;
    len = norm((random_point - near_point(1:2)),2);
    if(len < step)
        new_thea = random_point;
    else
        dir = (random_point - near_point(1:2))/len;
        new_thea = near_point(1:2) + dir*step;
    end
end
















