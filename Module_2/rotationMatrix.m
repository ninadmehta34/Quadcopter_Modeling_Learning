function rot = rotationMatrix(x_rot,y_rot,z_rot)

% for x-rotation
phi = deg2rad(x_rot);
Rx = [1 0         0       ; ...
    0 cos(phi) -sin(phi);...
    0 sin(phi)  cos(phi)];

% for y-rotation
theta = deg2rad(y_rot);  
Ry = [cos(theta) 0  sin(theta); ...
    0          1  0         ;...
    -sin(theta) 0  cos(theta)];

% for z-rotation
psi = deg2rad(z_rot);
Rz = [cos(psi) -sin(psi) 0;...
    sin(psi)  cos(psi) 0;...
    0         0        1];

% First rotate around the z-axis, then the y-axis, and finally the x-axis
rot = Rx*Ry*Rz;  

end

function ReturnHere() %#ok<DEFNU>
curloc = matlab.desktop.editor.getActiveFilename;
if contains(curloc,"Scripts")
    rootdir = extractBefore(curloc,filesep+"Scripts");
elseif contains(curloc,"InstructorResources")
    rootdir = extractBefore(curloc,filesep+"InstructorResources");
else
    error("Unexpected location: " + curloc)
end
cd(rootdir)
end