% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        samples -> num_samples x 4 matrix, vertices in the roadmap
%        adjacency -> num_samples x num_samples matrix, the weighted
%                     adjacency matrix denoting edges in the roadmap
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

function [path, path_found] = M3(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_centers, sphere_radii)    
    % Initialize variables
    % Add q_start and q_goal to samples and adjacency
    samples = [samples; q_start; q_goal];
    adjacency = [adjacency, zeros(size(adjacency, 1), 2)];
    adjacency = [adjacency; zeros(2, size(adjacency, 2))];
    E = [];

    % For each vertex, find num_neighbors closest neighbors
    for i = 101:102
        distances = zeros(1, 102);
        for j = 1:102
            if i == j
                distances(j) = inf;
            else
                % Compute Euclidean distance between vertices
                distances(j) = norm(samples(i, :) - samples(j, :));
            end
        end
        % Sort distances and get indices of closest neighbors
        [~, neighbor_indices] = sort(distances);
        neighbor_indices = neighbor_indices(1:10);
    
        % Check each edge for collision and add to adjacency matrix if collision-free
        for j = neighbor_indices
            if ~check_collision(robot, samples(i, :), link_radius, sphere_centers, sphere_radii) && ~check_collision(robot, samples(j, :), link_radius, sphere_centers, sphere_radii)
                if check_edge(robot, samples(i, :), samples(j, :), link_radius, sphere_centers, sphere_radii)
                    continue
                end
                E = [E; i j distances(j)];
            end
        end
    end

    % Construct adjacency matrix from edges
    for i = 1:size(E,1)
        adjacency(E(i,1), E(i,2)) = E(i,3);
        adjacency(E(i,2), E(i,1)) = E(i,3);
    end

    % Create a digraph object from the adjacency matrix and node names
    G = digraph(adjacency);

    % Find shortest path with the shortest path MATLAB function
    [shortest_path, path_length] = shortestpath(G, size(adjacency, 1)-1, size(adjacency, 1));

    % Check if a collision-free path was found
    if path_length == Inf
        path_found = false;
        path = [];
    else
        path_found = true;
        path = samples(shortest_path, :);
    end

end