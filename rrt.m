load("3DOFArm.mat", "robot")
load("collisionEnv.mat", "env")

figure
show(robot,Collisions="on");
hold on
for i = 1:length(env)
    show(env{i});
end

planner = manipulatorRRT(robot, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 0.1;
planner.ValidationDistance = 0.1;

startconfig = robot.homeConfiguration;
goalConfig = [-1.2; 0.4; -3.8];

rng(0);
path = plan(planner, startconfig', goalConfig');

interpStates = interpolate(planner, path);

for i = 1:size(interpStates, 1)
    show(robot, interpStates(i, :)', PreservePlot=false, Collisions="on");
    title("plan 1: MaxConnectionDistance = 0.3")
    drawnow
end




