clc
clear var
robot = loadrobot ( "kukaIiwa7", "DataFormat","row");
robot.Gravity = [0 0 -9.81];

for i = 1:robot.NumBodies
    clearCollision(robot.Bodies{i})
end

collisionObj = collisionCylinder(0.05,0.25);

for i = 1:robot.NumBodies
    if i > 6 && i < 10 
    else
        addCollision(robot.Bodies{i}, collisionObj)
    end
end

clearCollision(robot.Bodies{1})
clearCollision(robot.Bodies{2})

baseSize = [0.2 0.2 0.2];
baseBox = collisionBox(baseSize(1), baseSize(2), baseSize(3));
baseBox.Pose = trvec2tform([0 0 0]);
addCollision(robot.Bodies{1}, baseBox);

link1Radius = 0.03;   
link1Length = 0.15;
link1Cyl = collisionCylinder(link1Radius, link1Length);
link1Cyl.Pose = trvec2tform([0 0 link1Length/3]);

addCollision(robot.Bodies{2}, link1Cyl);

env = {collisionBox(0.75, 1, 0.1) collisionBox(0.5, 0.5, .8) collisionBox(0.8, 0.8, .3) collisionBox(0.1, 0.1, 1.5) collisionBox(0.1, 1.0, .1) collisionSphere(0.05) collisionSphere(0.05)};
env{1}.Pose(1:3, end) = [0.5 -0.65 0.4]; % Object plate
env{2}.Pose(1:3, end) = [0.4 0.7 0.3]; % Tall finish platform
env{3}.Pose(1:3, end) = [-0.6 -0.6 0.05]; % Short finish platform
env{4}.Pose(1:3, end) = [0.4 0.25 0.65]; % Vertical pillar obstacle
env{5}.Pose(1:3, end) = [-0.2 -0.7 0.75]; % Horizontal Pillar obstacle
env{6}.Pose(1:3, end) = [.7 -0.2 0.5]; % Sphere object
env{7}.Pose(1:3, end) = [.5 -0.2 0.5];% Sphere object


% Environment used for collision checking 
envPlan = {env{2}, env{3}, env{4}, env{5}};

% setup 
endEffector = robot.BodyNames{end};
ik = inverseKinematics('RigidBodyTree', robot);
ikWeights = [1 1 1 1 1 1];
ikInitGuess = homeConfiguration(robot);

qHome = homeConfiguration(robot);

% pick poses
pickOffset = [0 0 0.05];

obj1Pos = env{6}.Pose(1:3,4)';
obj2Pos = env{7}.Pose(1:3,4)';

R_ee = axang2rotm([1 0 0 pi]);
T_pick1 = trvec2tform(obj1Pos + pickOffset) * rotm2tform(R_ee);
T_pick2 = trvec2tform(obj2Pos + pickOffset) * rotm2tform(R_ee);

[qPick1, ~] = ik(endEffector, T_pick1, ikWeights, ikInitGuess);
[qPick2, ~] = ik(endEffector, T_pick2, ikWeights, ikInitGuess);

choice = input('Pick which object? (1 for env{6}, 2 for env{7}): ');

if choice == 1
    qPick = qPick1;
    objIdx = 6;
elseif choice == 2
    qPick = qPick2;
    objIdx = 7;
else
    error('Invalid choice. Enter 1 or 2.');
end

R_ee_place     = eye(3);                     % don't force strong orientation
ikWeightsPlace = [1 1 1 0.01 0.01 0.01];     % position >> orientation

% Ask user for desired target
targetXYZ = input('Enter desired place point [x y z] (e.g. [0.4 0.2 0.6]): ');
targetXYZ = double(targetXYZ);

% First try: exact point the user gave
T_place_des = trvec2tform(targetXYZ) * rotm2tform(R_ee_place);
[qPlace_try, solInfo] = ik(endEffector, T_place_des, ikWeightsPlace, qPick);

validDirect = false;
if solInfo.ExitFlag == 1
    inCollision = checkCollision(robot, qPlace_try, envPlan, ...
                                 'SkippedSelfCollisions','parent');
    if ~inCollision
        qPlace   = qPlace_try;
        T_place  = T_place_des;
        validDirect = true;
        fprintf('Using your exact requested point: [%.3f %.3f %.3f]\n', targetXYZ);
    end
end

if ~validDirect
    fprintf('Requested point is not directly usable. Searching nearby for closest valid point...\n');

    % Radial search around the user's point
    radii = 0.00:0.02:0.25;   % 0 to 25 cm away
    dirs  = [ 1 0 0;
             -1 0 0;
              0 1 0;
              0 -1 0;
              0 0 1;
              0 0 -1;
              1 1 0;
              1 -1 0;
              1 0 1;
              1 0 -1;
              0 1 1;
              0 1 -1];

    % normalize diagonals
    for d = 1:size(dirs,1)
        dirs(d,:) = dirs(d,:) / norm(dirs(d,:));
    end

    bestDist2 = inf;
    foundAny  = false;

    for r = radii
        for d = 1:size(dirs,1)
            candPos = targetXYZ + r * dirs(d,:);

            T_cand = trvec2tform(candPos) * rotm2tform(R_ee_place);
            [qCand, solInfoCand] = ik(endEffector, T_cand, ikWeightsPlace, qPick);

            if solInfoCand.ExitFlag ~= 1
                continue;
            end

            inCollision = checkCollision(robot, qCand, envPlan, ...
                                         'SkippedSelfCollisions','parent');
            if inCollision
                continue;
            end

            dist2 = sum((candPos - targetXYZ).^2);
            if dist2 < bestDist2
                bestDist2 = dist2;
                qPlace    = qCand;
                T_place   = T_cand;
                foundAny  = true;
            end
        end

        if foundAny
            break;   % stop at first radius where we find something
        end
    end

    if ~foundAny
        error('Could not find any nearby valid place point. Try a very different region.');
    else
        bestPos = tform2trvec(T_place);
        fprintf('Using nearest valid point: [%.3f %.3f %.3f]\n', bestPos);
    end
end

% Final safety check (should always be false now)
if checkCollision(robot, qPlace, envPlan, 'SkippedSelfCollisions','parent')
    error('Internal error: qPlace is still in collision with envPlan.');
end

% RRT planner
envPlan = {env{2}, env{3}, env{4}, env{5}};

planner1 = manipulatorRRT(robot, envPlan);
planner1.MaxConnectionDistance = 0.1;
planner1.ValidationDistance    = 0.02;
planner1.IgnoreSelfCollision   = true;
planner1.SkippedSelfCollisions = "parent";

[path1, solnInfo1] = plan(planner1, qHome, qPick);
if isempty(path1)
    error('No collision-free path found from home to pick pose.');
end

% Path 2
planner2 = manipulatorRRT(robot, envPlan);
planner2.MaxConnectionDistance = 0.1;
planner2.ValidationDistance    = 0.02;
planner2.IgnoreSelfCollision   = true;
planner2.SkippedSelfCollisions = "parent";

[path2, solnInfo2] = plan(planner2, qPick, qPlace);
if isempty(path2)
    error('No collision-free path found from pick pose to place pose.');
end

fullPath = [path1; path2(2:end,:)];

% ---------- Simulate grasp + animation ----------
% Smooth path with fewer frames and smooth interpolation
numFrames = 200;
s      = linspace(0, 1, size(fullPath,1));
sFine  = linspace(0, 1, numFrames);
fullPathSmooth = interp1(s', fullPath, sFine, 'pchip');

% Parameter where the pick segment ends (end of path1)
pickParam = s(size(path1,1));

% Figure + axes
figure; clf;
ax = gca; hold(ax,'on');
axis(ax,'equal');
axis(ax,[-1.5 1.5  -1.5 1.5  0 1.5]);
view(ax,135,25);
xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
title(ax,'KUKA iiwa Pick-and-Place');

platformColor = [1 0.5 0];   % orange
sphereColor   = [0 0.2 1];   % blue

% REAL EE->object transform at pick
T_ee_pick = getTransform(robot, qPick, endEffector);  
T_obj_pick = env{objIdx}.Pose;                        
T_ee_to_obj = T_ee_pick \ T_obj_pick;                 

for i = 1:size(fullPathSmooth,1)

    q = fullPathSmooth(i,:);

    % After reaching the pick portion of the path, attach the chosen sphere
    if sFine(i) >= pickParam
        T_ee = getTransform(robot, q, endEffector);
        env{objIdx}.Pose = T_ee * T_ee_to_obj;
    end

    % Redraw everything
    cla(ax); hold(ax,'on');

    % Draw environment: platforms + BOTH spheres (one may be moving)
    for k = 1:numel(env)
        [~, pObj] = show(env{k}, 'Parent', ax);   % 2nd output = patch

        if k == 6 || k == 7
            pObj.FaceColor = sphereColor;         % spheres
        else
            pObj.FaceColor = platformColor;       % platforms / pillars
        end
        pObj.EdgeColor = 'none';
    end

    % Draw robot with nice visuals
    show(robot, q, ...
        'Parent', ax, ...
        'Visuals',"on", ...
        'Collisions',"off", ...
        'PreservePlot', true);

    drawnow;
    pause(0.02);   % adjust for speed: smaller = faster, larger = slower
end