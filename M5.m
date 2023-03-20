% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

function smoothed_path = M5(robot, path, link_radius, sphere_centers, sphere_radii, path_edge)
    % Initialize variables
    smoothed_path = path(1, :); % add q_start to smoothed path
    current_idx = 1;
    next_idx = size(path, 1);
%     smoothed_path_edge = [];

    % Check each waypoint with each successive waypoint for collision-free path
    while current_idx < size(path, 1)
        % loop in reverse from q_goal to current vertex
        for i = size(path, 1):-1:next_idx
            q_near = path(current_idx, :);
            q_new = path(i, :);
            if ~check_collision(robot, q_near, link_radius, sphere_centers, sphere_radii) && ~check_collision(robot, q_new, link_radius, sphere_centers, sphere_radii)
                if check_edge(robot, q_near, q_new, link_radius, sphere_centers, sphere_radii)
                    continue
                end
                % if the edge is collision-free, remove all vertices in between
                smoothed_path = [smoothed_path; q_new];
%                 smoothed_path_edge = [smoothed_path_edge; path_edge(end, -1:)];
                current_idx = i;
                next_idx = size(path, 1);
                break
            end
        end
        
        % if the current vertex cannot reach the goal, try the next vertex
        if i == next_idx
            next_idx = current_idx + 1;
        end
    end

end