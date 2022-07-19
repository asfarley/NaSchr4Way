%Nagel-Schreckenberg signalized intersection model
%Alexander Farley
%alexander.farley at utoronto.ca
%March 27 2011
%------------------------------------------------------------------------------
%This model demonstrates an extension of the Nagel-Schreckenberg CA model to a
%4-way 6-phase signalized intersection (green, yellow, red from both
%directions). Both left and right turns are allowed during green. Right
%turns are also allowed during yellow and red. Low-priority traffic streams
%yield to high-priority streams.

%% Global simulation parameters
global road_length
global vmax
global p
global simulation_steps
global render
global verbose
global green_length
global yellow_length
global pause_length

global north_density
global south_density
global east_density
global west_density

global north_left_fraction
global north_right_fraction
global south_left_fraction
global south_right_fraction
global east_left_fraction
global east_right_fraction
global west_left_fraction
global west_right_fraction

global northbound_occupation_avg
global southbound_occupation_avg
global eastbound_occupation_avg
global westbound_occupation_avg

global northbound_velocity_avg
global southbound_velocity_avg
global eastbound_velocity_avg
global westbound_velocity_avg

global northbound_deletion_rate
global southbound_deletion_rate
global eastbound_deletion_rate
global westbound_deletion_rate

%% Timing parameters
error_count = 0; %For counting the number of car miscalculations. Should always be 0 - this is only for debugging.

%Generate timing signal
red_length = green_length + yellow_length + pause_length;
phaseNS = zeros(1,green_length+yellow_length+red_length+pause_length);
phaseEW = zeros(1,green_length+yellow_length+red_length+pause_length);

%NS Generate green
phaseNS(1:green_length) = ones(1,green_length);
%NS Generate yellow
phaseNS(green_length+1:green_length+yellow_length) = 2*ones(1,yellow_length);
%EW Generate green
phaseEW(green_length+yellow_length+pause_length+1:2*green_length+yellow_length+pause_length) = ones(1,green_length);
%EW Generate yellow
phaseEW(2*green_length+yellow_length+pause_length+1:2*green_length+2*yellow_length+pause_length) = 2*ones(1,yellow_length);
%% Generate intersection
%Using a layer-based model makes description of the geometry and
%boundary conditions simple. All qualitative differences are represented as
%occupancy on different layers.
map_size = 2*road_length + 2*vmax + 2; %One road approach on either side, one vmax-length padding per side, intersection of width 2

%Generate the intersection center
%1. Generate central square
center = zeros(map_size, map_size);
center(map_size/2:map_size/2+1, map_size/2:map_size/2+1) = 1;

%2. Generate vmax-length padding
padding = zeros(map_size, map_size);
padding(map_size/2 - vmax:map_size/2 - 1,map_size/2+1) = 1;           %South-north exit
padding(map_size/2+2:map_size/2 + 1 + vmax, map_size/2+1) = 1;      %South-north approach
padding(map_size/2 - vmax:map_size/2 - 1,map_size/2) =1;               %North-south approach
padding(map_size/2+2:map_size/2 + 1 + vmax, map_size/2) =1;          %North-south exit

padding(map_size/2,map_size/2 - vmax:map_size/2 - 1) = 1;              %East-west exit
padding(map_size/2,map_size/2+2:map_size/2+1+vmax) = 1;           %East-west approach
padding(map_size/2+1,map_size/2 -vmax:map_size/2 -1) = 1;             %West-east approach
padding(map_size/2+1,map_size/2 +2:map_size/2 +1+ vmax) = 1;        %West-east exit


%Generate the simple NaSchr outer lanes
lanes = zeros(map_size, map_size);
lanes(map_size/2+2+vmax:map_size/2 + 1+ vmax + road_length , map_size/2 + 1) = 1;   %South-north approach
lanes(map_size/2-vmax-road_length:map_size/2 -1 - vmax, map_size/2 +1) = 1;         %South-north exit
lanes(map_size/2-vmax-road_length:map_size/2 -1 - vmax, map_size/2) = 1;            %North-south approach
lanes(map_size/2+2+vmax:map_size/2 + 1+ vmax + road_length , map_size/2) = 1;       %North-south exit

lanes(map_size/2, map_size/2 + 2 + vmax: map_size/2 + 1 + vmax + road_length) = 1;  %East-west approach
lanes(map_size/2+1, map_size/2 + 2 + vmax: map_size/2 + 1 + vmax + road_length) = 1;%West-east exit
lanes(map_size/2+1, map_size/2 - vmax - road_length: map_size/2 - 1 - vmax) = 1;    %West-east approach
lanes(map_size/2, map_size/2 - vmax - road_length: map_size/2 - 1 - vmax) = 1;      %East-west exit


%Generate priority projection layers
high_priority = zeros(map_size, map_size);  %high priority layer: points on this array are occupied when a high priority vehicle will occupy them over the next timestep

%Generate turn decision layer
trajectory_decision = zeros(map_size, map_size);  % Intersection turn (trajectory) decision - this will be one of 12 codes defining the intended turn: 4x3 possibilities

%S->N: 1 S->E: 2 S-> W: 3
%N->S: 4 N->E: 5 N-> W: 6
%E->N: 7 E->S: 8 E-> W: 9
%W->N: 10 W->E: 11 W-> S: 12
northbound_trajectories =  [1 2 3 7 10]; %These arrays contain integer flags indicating turn decisions (used for easily detecting which movements apply to which vehicles)
southbound_trajectories =  [4 5 6 8 12];
eastbound_trajectories = [10 11 12 2 5];
westbound_trajectories = [7 8 9 3 6];

left_turn_trajectories = [3 8 4 10];
right_turn_trajectories = [2 7 6 12];
straight_through_trajectories = [1 9 4 11];

trajectory_decision_next = trajectory_decision;


%Generate "free distances" layer
distance_layer = zeros(map_size,map_size);

%Preallocate state sampling arrays. These are used to display a 2D plot of
%time on one axis and direction on the other axis, for each lane. This is
%the type of graphic normally used to display NaSchr simulations.
%This should probably be re-written to just sample and record the entire
%state of the system throughout the run.
northbound_occupation = zeros(simulation_steps,map_size);
southbound_occupation = zeros(simulation_steps,map_size);
westbound_occupation = zeros(simulation_steps,map_size);
eastbound_occupation = zeros(simulation_steps,map_size);

northbound_velocity = zeros(simulation_steps,map_size);
southbound_velocity = zeros(simulation_steps,map_size);
westbound_velocity = zeros(simulation_steps,map_size);
eastbound_velocity = zeros(simulation_steps,map_size);

occupation_layer = zeros(map_size, map_size); %Generate vehicle occupation layer
velocity_layer = zeros(map_size, map_size);   %Generate vehicle velocity layer
occupation_layer_next = zeros(map_size, map_size);
velocity_layer_next = zeros(map_size, map_size);

if render
    figure
    hold on
    h = gcf
    %set(gcf,'DoubleBuffer','on')
    %set(h,'erasemode','xor');
end

%% Iterate
for n=1:simulation_steps
%% Initialize/clear loop variables
    initial_count = sum(occupation_layer(:));           %Count all vehicles
    occupation_layer_next = 0*occupation_layer_next;    %Clear temp layer variables
    velocity_layer_next = 0*velocity_layer_next;
    trajectory_decision_next = 0*trajectory_decision_next;
    high_priority = 0*high_priority;
    distance_layer = 0*distance_layer;
    bf = 0;                                             %Set break flag to 1 to exit loop
%% Generate new vehicles at approaches
%Southbound approach
 if  rand < south_density && occupation_layer(1,map_size/2) == 0
        initial_count = initial_count + 1;
         occupation_layer(1,map_size/2) = 1;
         velocity_layer(1,map_size/2) = round(vmax*rand);
        turn_rand = rand;
        if turn_rand < south_left_fraction
        trajectory_decision(1,map_size/2) = 5;%N->E
        else
            if turn_rand < south_left_fraction + south_right_fraction
                trajectory_decision(1,map_size/2) = 6;%N->W
            else
                trajectory_decision(1,map_size/2) = 4;%N->S
            end
        end
 end

%Northbound approach
 if rand < north_density && occupation_layer(map_size,map_size/2+1) == 0
     initial_count = initial_count + 1;
         occupation_layer(map_size,map_size/2+1) = 1;
         velocity_layer(map_size,map_size/2+1) = round(vmax*rand);
        turn_rand = rand;
        if turn_rand < north_left_fraction
        trajectory_decision(map_size,map_size/2+1) = 3;%S->W
        else
            if turn_rand < north_left_fraction + north_right_fraction
                trajectory_decision(map_size,map_size/2+1) = 2;%S->E
            else
                trajectory_decision(map_size,map_size/2+1) = 1;%S->N
            end
        end
 end

%Westbound approach
        if rand < west_density  && occupation_layer(map_size/2,map_size) == 0 %Don't generate cars inside intersection
            initial_count = initial_count + 1;
         occupation_layer(map_size/2,map_size) = 1;
         velocity_layer(map_size/2,map_size) = round(vmax*rand);
        turn_rand = rand;
        if turn_rand < west_left_fraction
        trajectory_decision(map_size/2,map_size) = 8;%E->S
        else
            if turn_rand < west_left_fraction + west_right_fraction
                trajectory_decision(map_size/2,map_size) = 7;%E->N
            else
                trajectory_decision(map_size/2,map_size) = 9;%E->W
            end
        end
        end

%Eastbound approach
    if rand < east_density && occupation_layer(map_size/2+1,1) == 0 %Don't generate cars inside intersection
        initial_count = initial_count + 1;
         occupation_layer(map_size/2+1,1) = 1;
         velocity_layer(map_size/2+1,1) = round(vmax*rand);
        turn_rand = rand;
        if turn_rand < east_left_fraction
        trajectory_decision(map_size/2+1,1) = 10;%W->N
        else
            if turn_rand < east_left_fraction + east_right_fraction
                trajectory_decision(map_size/2+1,1) = 12;%W->S
            else
                trajectory_decision(map_size/2+1,1) = 11;%W->E
            end
        end
    end
%% Update signal phase
    stageNS = phaseNS(1+mod(n,length(phaseNS)));
    stageEW = phaseEW(1+mod(n,length(phaseEW)));
%% Southbound high priority
        for i=1:map_size%Calculate high priority distances
    if occupation_layer(i, map_size/2) == 1 &&  (trajectory_decision(i,map_size/2) == 4 || trajectory_decision(i,map_size/2) == 5)  && stageNS == 1 %Straight through or left on green: high priority path calculated first
        %Seek forward to find distance to next car
        found_next = 0; %Loop exit flag once "next vehicle ahead" is detected. Necessary to "fully" break from the loop.
        distance = vmax + 1;
        for j=i+1:i+vmax
                if j <= map_size
                    entering_intersection = (j >= map_size/2 && i < map_size/2);
                    intersection_occupied = trajectory_decision(map_size/2+1, map_size/2) == 10 ||  trajectory_decision(map_size/2+1, map_size/2) == 5 || trajectory_decision(map_size/2+1, map_size/2) == 11; %Left turn or straight-through vehicle  on other side of intersection
                    yielding = entering_intersection && intersection_occupied;
                    if occupation_layer(j, map_size/2) == 1    %NaSchr (avoid collision with car ahead)
                        distance = j-i; found_next = 1;
                    elseif trajectory_decision(i,map_size/2) == 5 && (j >= map_size/2 + 2) %Stopping to turn left
                        distance = j-i; found_next = 1;
                    elseif yielding
                        distance = j-i; found_next = 1;
                    else %Not blocked, project priority
                        %if trajectory_decision(i,map_size/2) == 4
                        high_priority(j,map_size/2) = 1;%Set priority marking to block conflicting lower priorities (highest priority)
                        %else
                        %high_priority(j,map_size/2) = 2;%Set priority marking to block conflicting lower priorities (2nd highest priority)
                        %end
                    end
                else %Implement deletion rate
                    if rand > southbound_deletion_rate
                        distance = map_size - i + 1;
                    end
                end
            if found_next, break, end
        end
        distance_layer(i, map_size/2) = distance;
    end
        end
%% Northbound high priority
        for i=1:map_size%Calculate high priority distances
    if occupation_layer(i, map_size/2+1) == 1 &&  (trajectory_decision(i,map_size/2+1) == 1 || trajectory_decision(i,map_size/2+1) == 3) && stageNS == 1 %Straight through or left on green: high priority path calculated first
        %Seek forward to find distance to next car
        distance = vmax + 1;
        found_next = 0;%Loop exit flag once "next vehicle ahead" is detected. Necessary to "fully" break from the loop.
        for j=i-1:-1:i-vmax
            if j > 0
                 entering_intersection = (j <= map_size/2+1 && i > map_size/2 + 1);
                 intersection_occupied = trajectory_decision(map_size/2, map_size/2+1) == 8 ||  trajectory_decision(map_size/2, map_size/2+1) == 3 ||  trajectory_decision(map_size/2, map_size/2+1) == 9; %Left turn or straight-through vehicle  on other side of intersection
                 yielding = entering_intersection && intersection_occupied;
            if occupation_layer(j, map_size/2+1) == 1
                distance = i-j; found_next = 1;
            elseif yielding
                distance = i-j; found_next = 1;
            elseif trajectory_decision(i, map_size/2+1) == 3 &&  (j <= map_size/2-1 ) %Turning left
                distance = i-j; found_next = 1;
            else
                %if trajectory_decision(i,map_size/2+1) == 1
                high_priority(j,map_size/2+1) = 1;%Set priority marking to block conflicting lower priorities (highest priority)
                %else
                %high_priority(j,map_size/2+1) = 2;%Set priority marking to block conflicting lower priorities (2nd highest priority)
                %end
            end
            else %Implement deletion rate
                if rand > northbound_deletion_rate
                    distance = i;
                end
            end
            if found_next, break, end
        end
        distance_layer(i, map_size/2+1) = distance;
    end
        end
%% Westbound high priority
        for i=1:map_size%Calculate high priority distances
    if occupation_layer(map_size/2,i) == 1 &&  (trajectory_decision(map_size/2,i) == 9 || trajectory_decision(map_size/2,i) == 8) && stageEW == 1 %Straight through or left on green: high priority path calculated first
        %Seek forward to find distance to next car
        distance = vmax + 1;
        found_next = 0;%Loop exit flag once "next vehicle ahead" is detected. Necessary to "fully" break from the loop.
        for j=i-1:-1:i-vmax
            if j > 0
                 entering_intersection = (j <= map_size/2+1 && i > map_size/2 + 1);
                 intersection_occupied = trajectory_decision(map_size/2, map_size/2) == 4 ||  trajectory_decision(map_size/2, map_size/2) == 8 ||  trajectory_decision(map_size/2, map_size/2) == 5; %Left turn or straight-through vehicle  on other side of intersection
                 yielding = entering_intersection && intersection_occupied;
            if occupation_layer(map_size/2,j) == 1
                distance = i-j; found_next = 1;
            elseif yielding
                distance = i-j; found_next = 1;
            elseif trajectory_decision(map_size/2,i) == 8 &&  (j == map_size/2-1) %Turning left
                distance = i-j; found_next = 1;
            else
                %if trajectory_decision(map_size/2,i) == 9
                high_priority(map_size/2,j) = 1;%Set priority marking to block conflicting lower priorities (highest priority)
                %else
                %high_priority(map_size/2,j) = 2;%Set priority marking to block conflicting lower priorities (2nd highest priority)
                %end
            end
                else %Implement deletion rate
                if rand > westbound_deletion_rate
                    distance = i;
                end
            end
            if found_next, break, end
        end
        distance_layer(map_size/2,i) = distance;
    end
        end
%% Eastbound high priority
        for i=1:map_size%Calculate high priority distances
    if occupation_layer(map_size/2+1,i) == 1 &&  (trajectory_decision(map_size/2+1,i) == 11 || trajectory_decision(map_size/2+1,i) == 10) && stageEW == 1 %Straight through or left on green: high priority path calculated first
        %Seek forward to find distance to next car
        distance = vmax + 1;
        found_next = 0;%Loop exit flag once "next vehicle ahead" is detected. Necessary to "fully" break from the loop.
        for j=i+1:i+vmax
            if j <= map_size
                 entering_intersection = (j >= map_size/2 && i < map_size/2 );
                 intersection_occupied = trajectory_decision(map_size/2+1, map_size/2+1) == 10 ||  trajectory_decision(map_size/2+1, map_size/2+1) == 1 ||  trajectory_decision(map_size/2+1, map_size/2+1) == 3; %Left turn or straight-through vehicle  on other side of intersection
                 yielding = entering_intersection && intersection_occupied;
            if occupation_layer(map_size/2+1,j) == 1
                distance = j-i; found_next = 1;
            elseif yielding
                distance = j-i; found_next = 1;
            elseif trajectory_decision(map_size/2+1,i) == 10 &&  (j >= map_size/2 + 2) %Turning left
                distance = j-i; found_next = 1;
            else
                %if trajectory_decision(map_size/2+1,i) == 11
                high_priority(map_size/2+1,j) = 1;%Set priority marking to block conflicting lower priorities (highest priority)
                %else
                %high_priority(map_size/2+1,j) = 2;%Set priority marking to block conflicting lower priorities (2nd highest priority)
                %end
            end
                        else %Implement deletion rate
                if rand > eastbound_deletion_rate
                    distance = map_size - i + 1;
                end
            end
            if found_next, break, end
        end
        distance_layer(map_size/2+1,i) = distance;
    end
        end
%% Southbound low priority
        for i=1:map_size%Calculate low priority distances
            is_southbound = ismember(trajectory_decision(i,map_size/2),southbound_trajectories);
            is_lowpriority = ~ismember(trajectory_decision(i,map_size/2),straight_through_trajectories) || ~(stageNS==1);
            if is_southbound && is_lowpriority %either not straight through, or not green light, also exclude vehicles that are actually from other roads and just passing through - specifically left-turning vehicles which pass through but should not be affected
                left = trajectory_decision(i,map_size/2) == 5;            %Turning left
                straight = trajectory_decision(i,map_size/2) == 4;      %Travelling straight
                right = trajectory_decision(i,map_size/2) == 6;          %Turning right
                red_or_yellow = stageNS == 0 || stageNS == 2;        %Light is yellow or red
                at_intersection = i == map_size/2-1;                       %Currently at intersection
                waiting_for_red = (left || straight) && red_or_yellow && at_intersection;
                intersection_occupied = trajectory_decision(map_size/2, map_size/2+1) == 3 ||  trajectory_decision(map_size/2, map_size/2+1) == 9 ||  trajectory_decision(map_size/2, map_size/2+1) == 8; %Left turn or straight-through vehicle  at 9 o'clock in intersection
                distance = vmax + 1;
                bf = 0;
                for j=i+1:i+vmax%Seek forward to find distance to next "reason to stop"
                    if j <= map_size
                occupied = occupation_layer(j, map_size/2) == 1;
                priority = (high_priority(j,map_size/2)~=0);
                entering_intersection = (j >= map_size/2 && i < map_size/2);
                arriving_at_stop = red_or_yellow && entering_intersection && ~at_intersection;
                yielding = entering_intersection && intersection_occupied && ~left;
                turning_left = left && (j >= map_size/2 + 2);
                turning_right = right && (j >= map_size/2+1);
                    if occupied || priority || arriving_at_stop || turning_left || turning_right || yielding || waiting_for_red
                        distance = j-i; bf = 1;
                    end
                    else %Implement deletion rate
                        if rand > southbound_deletion_rate
                            distance = map_size - i + 1;
                        end
                    end
                    if bf == 1, break, end
                end
                if distance_layer(i,map_size/2) < distance %This prevents overwriting an already-calculated high priority distance. If distance has not been calculated, distance_layer will be 0
                distance_layer(i,map_size/2) = distance;
                end
             end
        end
%% Northbound low priority
        for i=1:map_size%Calculate low priority distances
            is_northbound = ismember(trajectory_decision(i,map_size/2+1),northbound_trajectories);
            is_lowpriority = ~ismember(trajectory_decision(i,map_size/2+1),straight_through_trajectories) || ~(stageNS==1);
              if is_northbound && is_lowpriority %either not straight through, or not green light, also exclude vehicles that are actually from other roads and just passing through - specifically left-turning vehicles which pass through but should not be affected
                left = trajectory_decision(i,map_size/2+1) == 3;            %Turning left
                straight = trajectory_decision(i,map_size/2+1) == 1;      %Travelling straight
                right = trajectory_decision(i,map_size/2+1) == 2;          %Turning right
                red_or_yellow = stageNS == 0 || stageNS == 2;        %Light is yellow or red
                at_intersection = i == map_size/2+2;                       %Currently at intersection
                waiting_for_red = (left || straight) && red_or_yellow && at_intersection;
                intersection_occupied = trajectory_decision(map_size/2+1, map_size/2) == 11 || trajectory_decision(map_size/2+1, map_size/2) == 5 || trajectory_decision(map_size/2+1, map_size/2) == 10; %Left turn or straight-through vehicle  at 9 o'clock in intersection
                distance = vmax + 1;
                bf = 0;
                for j=i-1:-1:i-vmax%Seek forward to find distance to next "reason to stop"
                    if j > 0
                occupied = occupation_layer(j, map_size/2+1) == 1;
                priority = (high_priority(j,map_size/2+1)~=0);
                entering_intersection = (j <= map_size/2+1 && i > map_size/2 + 1);
                arriving_at_stop = red_or_yellow && entering_intersection && ~at_intersection;
                yielding = entering_intersection && intersection_occupied && ~left;
                turning_left = left && (j <= map_size/2-1 );
                turning_right = right && (j <= map_size/2);
                    if occupied || priority || arriving_at_stop || turning_left || turning_right || yielding || waiting_for_red
                        distance = i-j; bf=1;
                    end
                        else %Implement deletion rate
                        if rand > northbound_deletion_rate
                            distance = i;
                        end
                    end
                    if bf == 1, break, end
                end
                if distance_layer(i,map_size/2+1) < distance%This prevents overwriting an already-calculated high priority distance. If distance has not been calculated, distance_layer will be 0
                distance_layer(i,map_size/2+1) = distance;
                end
             end
        end
%% Westbound low priority
        for i=1:map_size%Calculate low priority distances
            is_westbound = ismember(trajectory_decision(map_size/2,i),westbound_trajectories);
            is_lowpriority = ~ismember(trajectory_decision(map_size/2,i),straight_through_trajectories) || ~(stageEW==1);
          if is_westbound && is_lowpriority %either not straight through, or not green light,, also exclude vehicles that are actually from other roads and just passing through - specifically left-turning vehicles which pass through but should not be affected
                left = trajectory_decision(map_size/2,i) == 8;            %Turning left
                straight = trajectory_decision(map_size/2,i) == 9;      %Travelling straight
                right = trajectory_decision(map_size/2,i) == 7;          %Turning right
                red_or_yellow = stageEW == 0 || stageEW == 2;        %Light is yellow or red
                at_intersection = i == map_size/2+2;                       %Currently at intersection
                waiting_for_red = (left || straight) && red_or_yellow && at_intersection;
                intersection_occupied = trajectory_decision(map_size/2+1, map_size/2+1) == 1 || trajectory_decision(map_size/2+1, map_size/2+1) == 10|| trajectory_decision(map_size/2+1, map_size/2+1) == 3; %Left turn or straight-through vehicle  at 9 o'clock in intersection
                distance = vmax + 1;
                bf = 0;
                for j=i-1:-1:i-vmax%Seek forward to find distance to next "reason to stop"
                    if j >0
                occupied = occupation_layer(map_size/2,j) == 1;
                priority = (high_priority(map_size/2,j)~=0);
                entering_intersection = (j <= map_size/2+1 && i > map_size/2 + 1);
                arriving_at_stop = red_or_yellow && entering_intersection && ~at_intersection;
                yielding = entering_intersection && intersection_occupied && ~left;
                turning_left = left && (j == map_size/2-1);
                turning_right = right && (j <= map_size/2);
                    if occupied || priority || arriving_at_stop || turning_left || turning_right || yielding || waiting_for_red
                        distance = i-j; bf=1;
                    end
                        else %Implement deletion rate
                        if rand > westbound_deletion_rate
                            distance = i;
                        end
                    end
                    if bf == 1, break, end
                end
                if distance_layer(map_size/2,i) < distance%This prevents overwriting an already-calculated high priority distance. If distance has not been calculated, distance_layer will be 0
                distance_layer(map_size/2,i) = distance;
                end
          end
        end
%% Eastbound low priority
        for i=1:map_size%Calculate low priority distances
        is_eastbound = ismember(trajectory_decision(map_size/2+1,i),eastbound_trajectories);
        is_lowpriority = ~ismember(trajectory_decision(map_size/2+1,i),straight_through_trajectories) || ~(stageEW==1);
            if  is_eastbound && is_lowpriority %either not straight through, or not green light, also exclude vehicles that are actually from other roads and just passing through - specifically left-turning vehicles which pass through but should not be affected
                left = trajectory_decision(map_size/2+1,i) == 10;            %Turning left
                straight = trajectory_decision(map_size/2+1,i) == 11;      %Travelling straight
                right = trajectory_decision(map_size/2+1,i) == 12;          %Turning right
                red_or_yellow = stageEW == 0 || stageEW == 2;        %Light is yellow or red
                at_intersection = i == map_size/2-1;                       %Currently at intersection
                waiting_for_red = (left || straight) && red_or_yellow && at_intersection;
                intersection_occupied = trajectory_decision(map_size/2, map_size/2) ==  4 || trajectory_decision(map_size/2, map_size/2) == 8 || trajectory_decision(map_size/2, map_size/2) ==  5;%Left turn or straight-through vehicle  at 9 o'clock in intersection
                distance = vmax + 1;
                bf = 0;
                for j=i+1:i+vmax%Seek forward to find distance to next "reason to stop"
                    if j <= map_size
                occupied = occupation_layer(map_size/2+1,j) == 1;
                priority = (high_priority(map_size/2+1,j)~=0);
                entering_intersection = (j >= map_size/2 && i < map_size/2 );
                arriving_at_stop = red_or_yellow && entering_intersection && ~at_intersection;
                yielding = entering_intersection && intersection_occupied && ~left;
                turning_left = left && (j >= map_size/2 + 2);
                turning_right = right && (j >= map_size/2 + 1);
                    if occupied || priority || arriving_at_stop || turning_left || turning_right || yielding || waiting_for_red
                        distance = j-i; bf=1;
                    end
                      else %Implement deletion rate
                        if rand > eastbound_deletion_rate
                            distance = map_size - i + 1;
                        end
                    end
                    if bf == 1, break, end
                end
                if distance_layer(map_size/2+1,i) < distance %This prevents overwriting an already-calculated high priority distance. If distance has not been calculated, distance_layer will be 0
                distance_layer(map_size/2+1,i) = distance;
                end
             end
        end
%% Southbound movement
        for i=1:map_size
            if occupation_layer(i,map_size/2)==1 && ismember(trajectory_decision(i,map_size/2),southbound_trajectories)
                distance = distance_layer(i,map_size/2);
       %1. Acceleration
        if velocity_layer(i,map_size/2) < vmax && distance > velocity_layer(i,map_size/2)+1
            velocity_layer(i,map_size/2) = velocity_layer(i,map_size/2)+1;
        end
        %2. Braking
        if distance <= velocity_layer(i,map_size/2)
            if distance > 0
            velocity_layer(i,map_size/2) = distance -1;
            else
                velocity_layer(i,map_size/2) = 0;
            end
        end
        %3. Randomization
        if rand < p && velocity_layer(i,map_size/2) > 0
        velocity_layer(i,map_size/2) = velocity_layer(i,map_size/2) -1;
        end
        %4. Motion
        if i+velocity_layer(i,map_size/2) <= map_size && ~(i== map_size/2 && trajectory_decision(i,map_size/2) == 6) && ~(i== map_size/2+1 && trajectory_decision(i,map_size/2) == 5) %If not moving out of bounds, and not turning right, and not turning left
        occupation_layer_next(i+velocity_layer(i,map_size/2),map_size/2) = 1;
        velocity_layer_next(i+velocity_layer(i,map_size/2),map_size/2) = velocity_layer(i,map_size/2);
        trajectory_decision_next(i+velocity_layer(i,map_size/2),map_size/2) = trajectory_decision(i,map_size/2);

        if occupation_layer(i+velocity_layer(i,map_size/2),map_size/2) == 1 && velocity_layer(i,map_size/2) ~= 0%Crash detection
            disp('Southbound crash at:')
            disp('Victim:')
            [i+velocity_layer(i,map_size/2) map_size/2]
            trajectory_decision(i+velocity_layer(i,map_size/2), map_size/2)
            disp('Fault:')
            [i map_size/2]
            trajectory_decision(i, map_size/2)
            disp('Velocity:')
            velocity_layer(i, map_size/2)
            disp('Headway:')
            distance_layer(i, map_size/2)
            disp('stageNS:')
            stageNS
            disp('stageEW:')
            stageEW
        %    bf = 1;
        end

        elseif i+velocity_layer(i,map_size/2) > map_size
                initial_count = initial_count - 1;
        end

            end
        end
%% Northbound movement
        for i=1:map_size
            if occupation_layer(i,map_size/2+1) == 1 && ismember(trajectory_decision(i,map_size/2+1),northbound_trajectories)
        distance = distance_layer(i,map_size/2+1);
        %1. Acceleration
        if velocity_layer(i,map_size/2+1) < vmax && distance > velocity_layer(i,map_size/2+1)+1
            velocity_layer(i,map_size/2+1) = velocity_layer(i,map_size/2+1)+1;
        end
        %2. Braking
        if distance <= velocity_layer(i,map_size/2+1)
            if distance > 0
            velocity_layer(i,map_size/2+1) = distance -1;
            else
                velocity_layer(i,map_size/2+1) = 0;
            end
        end
        %3. Randomization
        if rand < p && velocity_layer(i,map_size/2+1) > 0
        velocity_layer(i,map_size/2+1) = velocity_layer(i,map_size/2+1) -1;
        end
        %4. Motion
        if  i-velocity_layer(i,map_size/2+1) > 0 && ~(i== map_size/2 && trajectory_decision(i,map_size/2+1) == 3) && ~(i== map_size/2+1 && trajectory_decision(i,map_size/2+1) == 2)
        occupation_layer_next(i-velocity_layer(i,map_size/2+1),map_size/2+1) = 1;
        velocity_layer_next(i-velocity_layer(i,map_size/2+1),map_size/2+1) = velocity_layer(i,map_size/2+1);
        trajectory_decision_next(i-velocity_layer(i,map_size/2+1),map_size/2+1) = trajectory_decision(i,map_size/2+1);

        if occupation_layer(i-velocity_layer(i,map_size/2+1),map_size/2+1) == 1 && velocity_layer(i,map_size/2+1) ~= 0%Crash detection
            disp('Northbound crash')
            disp('Victim:')
            [i-velocity_layer(i,map_size/2+1) map_size/2+1]
            trajectory_decision(i-velocity_layer(i,map_size/2+1), map_size/2+1)
            disp('Fault:')
            [i map_size/2+1]
            trajectory_decision(i, map_size/2+1)
            disp('Velocity:')
            velocity_layer(i, map_size/2+1)
            disp('Headway:')
            distance_layer(i, map_size/2+1)
            disp('stageNS:')
            stageNS
            disp('stageEW:')
            stageEW
        %    bf = 1;
        end

        elseif i-velocity_layer(i,map_size/2+1) <= 0
            initial_count = initial_count - 1;
        end

            end
        end
%% Westbound movement
        for i=1:map_size
            if occupation_layer(map_size/2,i) == 1 && ismember(trajectory_decision(map_size/2,i),westbound_trajectories)
                distance = distance_layer(map_size/2,i);
       %1. Acceleration
        if velocity_layer(map_size/2,i) < vmax && distance > velocity_layer(map_size/2,i)+1
            velocity_layer(map_size/2,i) = velocity_layer(map_size/2,i)+1;
        end
        %2. Braking
        if distance <= velocity_layer(map_size/2,i)
            if distance > 0
            velocity_layer(map_size/2,i) = distance -1;
            else
                velocity_layer(map_size/2,i) = 0;
            end
        end
        %3. Randomization
        if rand < p && velocity_layer(map_size/2,i) > 0
        velocity_layer(map_size/2,i) = velocity_layer(map_size/2,i) -1;
        end
        %4. Motion
        if  i-velocity_layer(map_size/2,i) > 0 && ~(i == map_size/2 && trajectory_decision(map_size/2,i) == 8) && ~ (i== map_size/2+1 && trajectory_decision(map_size/2,i) == 7)
        occupation_layer_next(map_size/2,i-velocity_layer(map_size/2,i)) = 1;
        velocity_layer_next(map_size/2,i-velocity_layer(map_size/2,i)) = velocity_layer(map_size/2,i);
        trajectory_decision_next(map_size/2,i-velocity_layer(map_size/2,i)) = trajectory_decision(map_size/2,i);

        if occupation_layer(map_size/2,i-velocity_layer(map_size/2,i)) == 1  && velocity_layer(map_size/2,i) ~= 0%Crash detection
            disp('Westbound crash')
            disp('Victim:')
            [map_size/2 i-velocity_layer(map_size/2,i)]
            trajectory_decision(map_size/2, i-velocity_layer(map_size/2,i))
            disp('Fault:')
            [map_size/2 i]
            trajectory_decision(map_size/2, i)
            disp('Velocity:')
            velocity_layer(map_size/2, i)
            disp('Headway:')
            distance_layer(map_size/2, i)
            disp('stageNS:')
            stageNS
            disp('stageEW:')
            stageEW
           % bf = 1;
        end

        elseif  i-velocity_layer(map_size/2,i) <= 0
            initial_count = initial_count - 1;
        end

            end
        end
%% Eastbound movement
        for i=1:map_size
            if occupation_layer(map_size/2+1,i) == 1 && ismember(trajectory_decision(map_size/2+1,i),eastbound_trajectories)
                distance = distance_layer(map_size/2+1,i);
       %1. Acceleration
        if velocity_layer(map_size/2+1,i) < vmax && distance > velocity_layer(map_size/2+1,i)+1
            velocity_layer(map_size/2+1,i) = velocity_layer(map_size/2+1,i)+1;
        end
        %2. Braking
        if distance <= velocity_layer(map_size/2+1,i)
            if distance > 0
            velocity_layer(map_size/2+1,i) = distance -1;
            else
                velocity_layer(map_size/2+1,i) = 0;
            end
        end
        %3. Randomization
        if rand < p && velocity_layer(map_size/2+1,i) > 0
        velocity_layer(map_size/2+1,i) = velocity_layer(map_size/2+1,i) -1;
        end
        %4. Motion
        if  i+velocity_layer(map_size/2+1,i) <= map_size && ~(i== map_size/2 && trajectory_decision(map_size/2+1,i) == 12) && ~(i== map_size/2+1 && trajectory_decision(map_size/2+1,i) == 10)
        occupation_layer_next(map_size/2+1,i+velocity_layer(map_size/2+1,i)) = 1;
        velocity_layer_next(map_size/2+1,i+velocity_layer(map_size/2+1,i)) = velocity_layer(map_size/2+1,i);
        trajectory_decision_next(map_size/2+1,i+velocity_layer(map_size/2+1,i)) = trajectory_decision(map_size/2+1,i);

        if occupation_layer(map_size/2+1,i+velocity_layer(map_size/2+1,i)) == 1  && velocity_layer(map_size/2+1,i) ~= 0 %Crash detection
            disp('Eastbound crash')
            disp('Victim:')
           [map_size/2+1 i+velocity_layer(map_size/2+1,i)]
           trajectory_decision(map_size/2+1, i+velocity_layer(map_size/2+1,i))
           disp('Fault:')
           [map_size/2+1 i]
           trajectory_decision(map_size/2+1, i)
           disp('Velocity:')
           velocity_layer(map_size/2+1,i)
           disp('Headway:')
            distance_layer(map_size/2+1,i)
           disp('stageNS:')
            stageNS
            disp('stageEW:')
            stageEW
          %  bf = 1;
        end

        elseif i+velocity_layer(map_size/2+1,i) > map_size
            initial_count = initial_count - 1;
        end

            end
        end
%% Update and  state, perform checks, sample.

    %Update state
    occupied_layer_last = occupation_layer;
    occupation_layer = occupation_layer_next;
    velocity_layer_last = velocity_layer;
    velocity_layer = velocity_layer_next;
    trajectory_decision_last = trajectory_decision;
    trajectory_decision = trajectory_decision_next;


    if verbose
    %Recount all vehicles - check for discrepancies
    %Need to also account for created & destroyed vehicles, so at the
    %earlier points in the code when vehicles are created or destroyed,
    %initial_count is incremented or decremented
    final_count  = sum(occupation_layer(:)); %Vehicle count after update
    if initial_count ~= final_count %Simple collision detection - this misses collisions of high speed vehicles who don't land on the same spot
        warning('Collision detected')
        initial_count
        final_count
        error_count = error_count+1;
        bf = 1;
    end

%         switch stageNS
%         case 0
%             disp('NS Red')
%         case 1
%             disp('NS Green')
%         case 2
%             disp('NS Yellow')
%     end
%
%     switch stageEW
%         case 0
%             disp('EW Red')
%         case 1
%             disp('EW Green')
%         case 2
%             disp('EW Yellow')
%     end

    end

    %Sample occupation state
    northbound_occupation(n,:) = occupation_layer(:,map_size/2+1)';
    southbound_occupation(n,:) = occupation_layer(:,map_size/2)';
    westbound_occupation(n,:) = occupation_layer(map_size/2,:);
    eastbound_occupation(n,:) = occupation_layer(map_size/2+1,:);

    %Sample velocity state
    northbound_velocity(n,:) = velocity_layer(:,map_size/2+1)';
    southbound_velocity(n,:) = velocity_layer(:,map_size/2)';
    westbound_velocity(n,:) = velocity_layer(map_size/2,:);
    eastbound_velocity(n,:) = velocity_layer(map_size/2+1,:);

    northbound_occupation_avg = northbound_occupation_avg + sum(northbound_occupation(:))/(simulation_steps*length(northbound_occupation(:)));
    southbound_occupation_avg = southbound_occupation_avg + sum(southbound_occupation(:))/(simulation_steps*length(southbound_occupation(:)));
    eastbound_occupation_avg = eastbound_occupation_avg + sum(eastbound_occupation(:))/(simulation_steps*length(eastbound_occupation(:)));
    westbound_occupation_avg = westbound_occupation_avg + sum(westbound_occupation(:))/(simulation_steps*length(westbound_occupation(:)));


    if sum(northbound_occupation(:)) > 0
    northbound_velocity_avg = northbound_velocity_avg + sum(northbound_velocity(:))/(simulation_steps*sum(northbound_occupation(:)));
    end

    if sum(southbound_occupation(:)) > 0
    southbound_velocity_avg = southbound_velocity_avg + sum(southbound_velocity(:))/(simulation_steps*sum(southbound_occupation(:)));
    end

    if sum(eastbound_occupation(:)) > 0
    eastbound_velocity_avg = eastbound_velocity_avg + sum(eastbound_velocity(:))/(simulation_steps*sum(eastbound_occupation(:)));
    end

    if sum(westbound_occupation(:)) > 0
    westbound_velocity_avg = westbound_velocity_avg + sum(westbound_velocity(:))/(simulation_steps*sum(westbound_occupation(:)));
    end

    if render
    %Render intersection
    clf
    h = imshow(lanes+center+padding-occupation_layer, 'InitialMagnification', 'fit');
    %set(gcf,'DoubleBuffer','on')
    %set(h,'erasemode','xor');
    %screen_size = get(0, 'ScreenSize');
    %f1 = figure(1);
    %set(f1, 'Position', [0 0 screen_size(3) screen_size(4) ] );
    drawnow
    %pause(.01)
    end

    if testing
        waitforbuttonpress
    end

  %  if bf == 1, break, end
end



