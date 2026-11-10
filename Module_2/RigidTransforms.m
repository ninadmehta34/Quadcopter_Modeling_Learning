% Understand and apply rigid body transformations. 
% needs point cloud for representation purposes and nothing else. 
 % a few notes:

%  As a product of matrices, these transforms are non-commutative. 
% That is, the order in which they are performed matters. 
% If we rotate first around the z-axis, then the y-axis, then the x-axis, 
% we'll end up in a different orientation than if we rotate around the x-axis first, 
% then the y-axis, then the z-axis. 



% When specifying 3D rotations using Euler angles, 
% it is important to distinguish between extrinsic and intrinsic rotation order. 
% In extrinsic rotations, each rotation is applied about the fixed, global axes 
% (e.g., rotate about global , then global , then global ). 
% In intrinsic rotations, each rotation is applied about the body-fixed axes, 
% meaning each subsequent rotation is about an axis that has already been rotated.
% In rigidtform3d, Euler angles are interpreted as intrinsic rotations about the x, then the y, then the z axes of the rotating frame. 
% Internally, this results in a rotation matrix of the form R =
% Rz(Sai)*Ry(theta)*Rx(Phi)
%Correspond to rotations about x, then y', then z". 
% This is mathematically equivalent to an extrinsic z-y-x rotation applied
% about fixed axes


% For navigation, Intrinsic Computations make more sense as the object
% measures its own location and orientation in space. 
% This makes it simpler to track "turn right" for example, rather than
% determining which angle corresponds to the "turn right" w.r.t fixed
% global. 

%%% SIMPLE Exercise first %%

ptCloud = pcread('teapot.ply');
figure
pcshow(ptCloud)
lim = 5;
DecoratePlot(lim)

function DecoratePlot(lim)
% Standardize the labels and limits of the output plot
xlabel("X")
xlim([-lim lim])
ylabel("Y")
ylim([-lim lim])
zlabel("Z")
zlim([-lim lim])
end

x_rot = 30; % degrees from -180 to 180
y_rot = 45; % degrees from -180 to 180
z_rot = 60; % degrees from -180 to 180
rot = rotationMatrix(x_rot,y_rot,z_rot);


x = 2;
y = -2;
z = 4;
trans = [x, y, z];

tform = rigidtform3d(rot,trans)

tform1 = rigidtform3d(rotationMatrix(x_rot,0,0),[x y z]);
tform2 = rigidtform3d(rotationMatrix(0,y_rot,0),[0 0 0]);
tform3 = rigidtform3d(rotationMatrix(0,0,z_rot),[0 0 0]);

figure; % X rotation + translation
ptCloudOut1 = pctransform(ptCloud,tform1);
pcshow(ptCloudOut1);
DecoratePlot(lim)


figure; % Y rotation
ptCloudOut2 = pctransform(ptCloudOut1,tform2);
pcshow(ptCloudOut2)
DecoratePlot(lim)

figure; % Z rotation
ptCloudOut3 = pcshow(pctransform(ptCloudOut2,tform3));
DecoratePlot(lim)

% COMBINED 
ptCloudOut = pctransform(ptCloud,tform);

figure
teapot = pcshow(ptCloudOut);
DecoratePlot(lim)


% Second Part of the exercise
% Use a for loop to create an animation that moves the teapot from the 
% original position to the new position over a reasonable number of steps 
% to appear smooth but not too slow. 

% Define the number of steps to use
iteration_number = 100;  % Change the value of 1 to the number of steps you want

% Define a path from the origin to your endpoint in each coordinate, 
%     such as by defining linearly spaced steps

% Rotation around x-axis
x_rot_steps = linspace(0,x_rot, iteration_number);
% Rotation around y-axis
y_rot_steps = linspace(0, y_rot, iteration_number);
% Rotation around z-axis
z_rot_steps = linspace(0, z_rot, iteration_number);
% Translation in the x direction
x_steps = linspace(0,x, iteration_number);
% Translation in the y direction
y_steps = linspace(0,y,iteration_number);
% Translation in the z direction
z_steps = linspace(0,z,iteration_number);

% Plot your original point cloud
tp = pcshow(ptCloud);  % Don't change this line
xlabel("X")
xlim([min(0,x)-3.5,max(0,x)+3.5])
ylabel("Y")
ylim([min(0,y)-3.5,max(0,y)+3.5])
zlabel("Z")
zlim([min(0,z)-1.5, max(0,z)+4])


for idx = 1:iteration_number
    % Generate the rigid transform of your figure at idx steps into the
    % animation and generate a new point cloud called ptCloudOut to replace
    % the default value of ptCloud
    rot = rotationMatrix(x_rot_steps(idx), y_rot_steps(idx), z_rot_steps(idx));
    trans = [x_steps(idx), y_steps(idx), z_steps(idx)];
    tform = rigidtform3d(rot, trans)


    % Point Cloud after this transformation

    ptCloudOut = pctransform(ptCloud, tform);  % Change ptCloud to be the transformed point cloud

    % This code updates the points in the diagram by re-using the original
    % teapot plot tp.
    tp.Children.XData = ptCloudOut.Location(:,1); % Do not change
    tp.Children.YData = ptCloudOut.Location(:,2); % Do not change
    tp.Children.ZData = ptCloudOut.Location(:,3); % Do not change
    % pcshow(ptCloudOut)
    pause(0.05); % Pause for a brief moment to visualize the rotation
end