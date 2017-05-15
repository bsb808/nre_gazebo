classdef pioneerwaypointclass < handle
   properties
       % ROS communication properties
       msg_twist  % Twist command to be published
       pub_twist  % Twist publisher
       subs  % cell array of subscribers

       % Enabled - does it publish
       enabled
       
       % Counters
       odomN
       geonavN
       statusN
              
       % Robot State
       x_odom;
       y_odom;
       th_odom;
       cnt_odom;
       x_geonav;
       y_geonav;
       th_geonav;
       cnt_geonav;
       filter_status;
       
       % Waypoint control properties
       waypoints
       wpt_index
       dist_threshold
       
       % Followers, cell array
       followers
       
       % Other
       name
   end
   methods
      % Initialization
      % This gets called when you instantiate the object.
      function init(obj,odom_topic,geonav_topic,...
              navstatus_topic,twist_topic,waypoints)
          % Default name
          obj.name='R0';
          
          % Start disabled
          obj.enabled = 0;
          
          % Distance to waypoint threshold 
          obj.dist_threshold = 1.0;
          
          % Initialize robot state
          obj.x_odom = nan;
          obj.y_odom = nan;
          obj.th_odom = nan;
          obj.cnt_odom = 0;  % counter
          obj.x_geonav = nan;
          obj.y_geonav = nan;
          obj.th_geonav = nan;
          obj.cnt_geonav = 0;
          obj.filter_status = nan;
          
          % Setup publishers 
          fprintf('Publishing Twist messages on <%s>\n',twist_topic);
          obj.pub_twist = rospublisher(twist_topic,'geometry_msgs/Twist');
          obj.msg_twist= rosmessage(obj.pub_twist);
          pause(2);
          
          % Setup subscriptions
          obj.subs = {};
          fprintf('Subscribing to odometry Odometry messages on <%s>\n',odom_topic);
          obj.subs{1} = rossubscriber(odom_topic,'nav_msgs/Odometry',@obj.odom_callback);
          fprintf('Subscribing to geonav Odometry messages on <%s>\n',geonav_topic);
          obj.subs{2} = rossubscriber(geonav_topic,'nav_msgs/Odometry',@obj.geonav_callback);
          fprintf('Subscribing to nav status messages on <%s>\n',navstatus_topic);
          obj.subs{3} = rossubscriber(navstatus_topic,'std_msgs/Int16MultiArray',@obj.navstatus_callback);
          
          % Add list of waypoints as a parameter
          obj.set_waypoints(waypoints);
                    
          % Followers
          obj.followers = {};
      end
      
      % Destructor
      function delete(obj)
          % delete all the subscriptoins
          disp('Deleting subscriptions')
          for ii = 1:length(obj.subs)
              disp(ii);
              %delete(obj.subs{ii});
              clear obj.subs;
          end
          pause(2);
          disp('That''s all folks...');
      end
      
      % Add a follower object to our list of followers
      function add_follower(obj,follower_obj)
          obj.followers{length(obj.followers)+1}=follower_obj;
      end
      
      function set_waypoints(obj,waypoints)
          obj.waypoints = waypoints;
          obj.wpt_index = 1;
          if size(obj.waypoints,1) < 1
              obj.wpt_index = 0;
          end
      end
      

      % Display status
      function display_status(obj)
          fprintf('Name=%s: odom[%d]: x=%4.1f y=%4.1f th=%4.1f; geonav[%d]: x=%4.1f, y=%4.1f, th=%4.1f filter_status=%d \n',...
              obj.name, obj.cnt_odom, obj.x_odom, obj.y_odom, obj.th_odom, ...
              obj.cnt_geonav, obj.x_geonav, obj.y_geonav, obj.th_geonav, obj.filter_status);
          if ( (obj.wpt_index > 0) && (size(obj.waypoints,1) > 0) )
              wp_x = obj.waypoints(obj.wpt_index,1);
              wp_y = obj.waypoints(obj.wpt_index,2);
          else
              wp_x = nan;
              wp_y = nan;
          end
          fprintf('\tCurrent waypoint[%d of %d]: X=%5.2f, Y=%5.2f; cmd_vel: linear=%4.2f, angular=%4.2f \n',...
              obj.wpt_index,size(obj.waypoints,1),wp_x,wp_y,obj.msg_twist.Linear.X,obj.msg_twist.Angular.Z);
      end
      
      % Odometry callback
      function odom_callback(obj,src,msg)
          obj.cnt_odom = obj.cnt_odom +1;
          [obj.x_odom,obj.y_odom,obj.th_odom]=obj.odom2xyt(msg);
                    
          %obj.odom2xyt(msg);
          %obj.test(msg)
      end
  
      % Geonav callback
      function geonav_callback(obj,src,msg)
          obj.cnt_geonav = obj.cnt_geonav + 1;
          [obj.x_geonav,obj.y_geonav,obj.th_geonav]=obj.odom2xyt(msg);
          
          % Set follower waypoints
          obj.set_follower_waypoints(obj.x_geonav,obj.y_geonav,obj.th_geonav);
         
          % Implement waypoint guidance
          obj.drive2waypoint(obj.x_geonav,obj.y_geonav,obj.th_geonav);
          
      end
      
      % Nav Status callback
      function navstatus_callback(obj,src,msg)
          status = msg.Data;
          obj.filter_status = status(1);
      end
      function rfid_callback(obj,src,msg)
          % pass
      end
      
      function set_follower_waypoints(obj,x,y,th)
          d = 0.5; % separation distance
          % angular spacing depending on number of followers
          da = pi/(1+length(obj.followers));
           % Notify followers of new pose
          for ii = 1:length(obj.followers)
              a = pi/2+da*ii;
              dd = ii*d;
              wp_x = x+dd*cos(a)*cos(th)-dd*sin(a)*sin(th);
              wp_y = y+dd*cos(a)*sin(th)+dd*sin(a)*cos(th);
              follower_ptr = obj.followers{ii};
              follower_ptr.set_waypoints([wp_x,wp_y]);
          end
      end
          
      % Function to convert Odometry to x, y, theta
      function [x,y,th] = odom2xyt(obj,msg)
          x = msg.Pose.Pose.Position.X;
          y = msg.Pose.Pose.Position.Y;
          q = msg.Pose.Pose.Orientation;
          angles = quat2eul([q.W,q.X,q.Y,q.Z]);
          th = angles(1);  % In radians!
      end
      
      % Drive to the current waypoint with the given position estimate
      function drive2waypoint(obj,x,y,th)
          if ( (obj.wpt_index == 0) || (obj.wpt_index > size(obj.waypoints,1)) )
              obj.publish_cmd_vel(0,0);
          else
              wp_x = obj.waypoints(obj.wpt_index,1);
              wp_y = obj.waypoints(obj.wpt_index,2);
              [dist,angvel,linvel] = obj.wcontrol(x,y,th,wp_x,wp_y);
              %disp(dist);
              if (dist < obj.dist_threshold)
                  obj.wpt_index = min(obj.wpt_index+1,size(obj.waypoints,1));
                  obj.publish_cmd_vel(0,0);
              else
                  obj.publish_cmd_vel(linvel,angvel);
              end
          end
      end
      
      function publish_cmd_vel(obj,lvel,avel)
          obj.msg_twist.Linear.X = lvel;
          obj.msg_twist.Angular.Z = avel;
          if (obj.enabled)
              send(obj.pub_twist,obj.msg_twist);
          end
      end
      % Regular methods
      function [dist,angvel,linvel] = wcontrol(obj,x,y,th,wp_x,wp_y)
          % Function to implement waypoint guidance
          % Inputs:
          %  - x: Current X position
          %  - y: Current Y position
          %  - th: Current theta position
          %  - wp_x: Current waypoint goals, X
          %  - wp_y: Current waypoint goals, X
          % Outputs:
          %  - dist: Distance from current position to current waypoint goal
          %  - angvel: Desired angular velocity [rad/s, clockwise]
          %  - linvel: Desired linear velocity
          
      
          % Position Error - distance to waypoint
          errorPose=sqrt((x-wp_x)^2+(y-wp_y)^2);
          
          % Theta Error
          goaltheta=(atan2((wp_y-y),(wp_x-x)));
          errortheta=goaltheta-th;
          
          % Clamp angle
          while errortheta > pi
              errortheta=errortheta-2*pi;
          end    
          while errortheta<-pi;
              errortheta=errortheta+2*pi;
          end
          dist=errorPose;
          ka = 2.0; % angle gain 
          angvel= ka*errortheta;
          kl = 0.5; % linear gain
          vel=kl*errorPose;
          linvel = sign(vel)*min(abs(vel),1.0);
      end
   end
end




