% Input: q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples to sample
% Output: qs -> num_samples x 4 matrix of joint angles,
%               all within joint limits

function qs = M1(q_min, q_max, num_samples)
    % initialize the matrix of joint angles
    qs = zeros(num_samples, 4);

    % loop to fill in values for matrix
    for i = 1:num_samples
        % generate a random configuration
        q = q_min + rand(1, 4) .* (q_max - q_min);
        % place q in proper position in matrix
        qs(i, :) = q;
    end

end