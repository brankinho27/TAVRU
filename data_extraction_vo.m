t_poses = zeros(height(poses(vSet)), 3);
r_poses = zeros(height(poses(vSet)), 9);

for i=1:height(poses(vSet))
    t_pose = poses(vSet).AbsolutePose(i,:).Translation;
    t_poses(i,:) = t_pose;
    
    r_pose = poses(vSet).AbsolutePose(i,:).Rotation;
    r_pose = r_pose';
    r_pose = r_pose(:)';
    r_poses(i,:) = r_pose;
end

writematrix(t_poses, 't_poses_vo.csv');
writematrix(r_poses, 'r_poses_vo.csv');