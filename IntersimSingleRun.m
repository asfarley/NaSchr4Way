global road_length
global vmax
global p
global simulation_steps
global render
global verbose

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

global northbound_deletion_rate
global southbound_deletion_rate
global eastbound_deletion_rate
global westbound_deletion_rate

render = 1;                   %Set to 1 to render state during simulation
verbose = 0;                  %Set to 1 to print state information during simulation
testing = 0;                  %Set to 1 to pause after each iteration

%NaSchr parameters
road_length = 100;                %The length of each approach to the intersection (governed by NaSchr, not included in intersection)
vmax = 5;                        %The maximum velocity in cells/timestep
p = 0.2;                         %parameter in NaSchr model for randomized braking
simulation_steps = 500;         %Length of simulation

%Timing parameters
green_length = 50;             %# simulation steps of the green light
yellow_length = 0;            %# simulation steps of the yellow light
pause_length = 0;             %# simulation steps of the red pause (when both lights are red)

north_left_fraction =  0.25;
north_right_fraction = 0.25;
south_left_fraction =  0.25;
south_right_fraction = 0.25;
east_left_fraction =   0.25;
east_right_fraction =  0.25;
west_left_fraction =   0.25;
west_right_fraction =  0.25;

density = .15;
north_density = density;
south_density = density;
east_density = density;
west_density = density;

northbound_deletion_rate = 1;
southbound_deletion_rate = 1;
eastbound_deletion_rate =  1;
westbound_deletion_rate =  1;

Intersim
Analyze
