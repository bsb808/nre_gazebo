
% Example of using Pioneer Waypoint Class for single robot waypoint
% guidance

clear

% Try to start ROS - if it is already started, restart
try
    rosinit
catch
    rosshutdown
    rosinit
end

% Setup the parameters to pass to the object
robot='p0';
odom_topic = sprintf('/%s/my_p3at/pose',robot);
geonav_topic = sprintf('/%s/geonav_odom',robot);
navstatus_topic = sprintf('/%s/nav/status',robot);
twist_topic = sprintf('/%s/my_p3at/cmd_vel',robot);

% Instatiate the object
leader = pioneerwaypointclass();

% Initialize with the parameters - this will start running immediately.
leader.init(odom_topic,geonav_topic,navstatus_topic,twist_topic,[]);
leader.name='leader';

% Give it some waypoints
leader.dist_threshold = 0.5;
leader.set_waypoints([95,26; 85,16; 90 20]);


% Add followers
n=2;
followers = {};
for ii = 1:n
    robot=sprintf('p%d',ii);
    odom_topic = sprintf('/%s/my_p3at/pose',robot);
    geonav_topic = sprintf('/%s/geonav_odom',robot);
    navstatus_topic = sprintf('/%s/nav/status',robot);
    twist_topic = sprintf('/%s/my_p3at/cmd_vel',robot);
    followers{ii} = pioneerwaypointclass();
    followers{ii}.init(odom_topic,geonav_topic,navstatus_topic,twist_topic,[]);
    followers{ii}.name=sprintf('follower%d',ii);
    followers{ii}.dist_threshold = 0.5;
    % Tell the leader about the follower
    if ii == 1
        leader.add_follower(followers{ii});
    else
        followers{ii-1}.add_follower(followers{ii});
    end
    
end


% Enable - this starts the publications
leader.enabled = 1;
for ii=1:length(followers)
    followers{ii}.enabled = 1;
end

% Register the cleanup function so that things exit gracefully
cleanupObj = onCleanup(@()rosshutdown);

while true
    leader.display_status();
    for ii=1:length(followers)
        followers{ii}.display_status();
    end
    pause(1.0)
end

