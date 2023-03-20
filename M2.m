% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples in PRM
%        num_neighbors -> Integer denoting number of closest neighbors to
%                         consider in PRM
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: samples -> num_samples x 4 matrix, sampled configurations in the
%                    roadmap (vertices)
%         adjacency -> num_samples x num_samples matrix, the weighted
%                      adjacency matrix denoting edges between roadmap
%                      vertices. adjacency(i,j) == 0 if there is no edge
%                      between vertex i and j; otherwise, its value is the
%                      weight (distance) between the two vertices. For an
%                      undirected graph, the adjacency matrix should be
%                      symmetric: adjacency(i,j) == adjacency(j,i)

function [samples, adjacency] = M2(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_centers, sphere_radii)
    % Initialize samples and adjacency matrix
    samples = zeros(num_samples, 4);
    adjacency = zeros(num_samples);
    
    % Initialize empty set of vertices and edges
    V = [];
    E = [];
    
    % Sample num_samples configurations within joint limits
    while size(V,1) < num_samples
        q = rand(1,4) .* (q_max - q_min) + q_min;
        if check_collision(robot, q, link_radius, sphere_centers, sphere_radii)
            continue
        end
        V = [V; q];
    end
    
    % For each vertex, find num_neighbors closest neighbors
    for i = 1:num_samples
        distances = zeros(1, num_samples);
        for j = 1:num_samples
            if i == j
                distances(j) = inf;
            else
                % Compute Euclidean distance between vertices
                distances(j) = norm(V(i, :) - V(j, :));
            end
        end
        % Sort distances and get indices of closest neighbors
        [~, neighbor_indices] = sort(distances);
        neighbor_indices = neighbor_indices(1:num_neighbors);
    
        % Check each edge for collision and add to adjacency matrix if collision-free
        for j = neighbor_indices
            if ~check_collision(robot, V(i, :), link_radius, sphere_centers, sphere_radii) && ~check_collision(robot, V(j, :), link_radius, sphere_centers, sphere_radii)
                if check_edge(robot, V(i, :), V(j, :), link_radius, sphere_centers, sphere_radii)
                    continue
                end
                E = [E; i j distances(j)];
            end
        end
    end

    % Extract the indices of the connected vertices from the edges and remove duplicates
    edge_vertices = unique([E(:,1); E(:,2)]);

    % Use the vertex indices to get the actual samples
    samples = V(edge_vertices, :);
    
    % Construct adjacency matrix from edges
    for i = 1:size(E,1)
        adjacency(E(i,1), E(i,2)) = E(i,3);
        adjacency(E(i,2), E(i,1)) = E(i,3);
    end

    
end