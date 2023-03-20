% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found

function [path, path_found, V, E, path_edge] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    % Initialize variables
    number_of_nodes = 500;
    alpha = 0.1;
    beta = 0.1;

    % Initialize the tree
    V = q_start;
    E = [];

    % Build the tree
    for i = 1:number_of_nodes
        % Decide whether to sample the goal configuration or a random one
        if rand(1) < beta
            q_target = q_goal;
        else
            q_target = q_min + rand(1, 4) .* (q_max - q_min);
        end

        % Find the nearest neighbor to q_target
        distances = vecnorm(V - q_target, 2, 2);
        [~, nearest_idx] = min(distances);
        q_near = V(nearest_idx, :);

        % Extend the tree towards the random configuration
        q_new = q_near + (alpha / norm(q_target - q_near)) * (q_target - q_near);

        % Check for collision along the edge (q_near, q_new)
        if ~check_collision(robot, q_near, link_radius, sphere_centers, sphere_radii) && ~check_collision(robot, q_new, link_radius, sphere_centers, sphere_radii)
            if check_edge(robot, q_near, q_new, link_radius, sphere_centers, sphere_radii)
                continue
            end
            % Add the new configuration to the tree
            V = [V; q_new];
            E = [E; nearest_idx, size(V, 1)];
        end
    end
    
    % Connect q_goal to the nearest vertex in the tree
    distances = vecnorm(V - q_goal, 2, 2);
    [~, nearest_idx] = min(distances);
    q_near_goal = V(nearest_idx, :);
    
    % Find the nearest edge that doesn't have collisions
    path_found = false;
    for i = 1:size(E, 1)
        q1 = V(E(i, 1), :);
        q2 = V(E(i, 2), :);
        if check_edge(robot, q1, q_goal, link_radius, sphere_centers, sphere_radii) || ...
                check_edge(robot, q_goal, q2, link_radius, sphere_centers, sphere_radii)
            continue
        else
            E = [E; E(i, 1), size(V, 1)+1; size(V, 1)+1, E(i, 2)];
            V = [V; q_goal];
            path_found = true;
            break
        end
    end
    
    if ~path_found
        % No path found
        path = [];
        path_edge = [];
    else
        % Traverse the tree from q_start to q_goal to get the path
        path = [q_goal];
        path_edge = [];
    
        % Build the tree from goal to path
        % Trace back from q_goal to q_start and save to path
        current_idx = size(V, 1);
        while current_idx ~= 1
            for i = 1:size(E, 1)
                if E(i, 2) == current_idx
                    current_idx = E(i, 1);
                    path = [V(current_idx, :); path];
                    path_edge = [E(i, :); path_edge];
                    break
                end
            end
        end
        path = [q_start; path];
%         path_edge = [path_edge; path_edge(end, 2), E(end, 2)];
    end

end