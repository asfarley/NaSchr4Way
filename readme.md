4-Way Intersection Simulation based on Nagel-Schreckenberg traffic model
------------------------------------------------------------------------

This MATLAB/Octave script contains a simulation model of a 4-way intersection based on the Nagel-Schreckenberg single lane traffic model. This script extends the NaSChr model to provide rules for vehicles passing through a signalized 4-way intersection. The intersection allows the possibility of left and right turns for a total of 12 possible movements from each approach to each exit. 

In order to see results, run the `InterSimSingleRun.m` script. 

The Nagel-Schreckenberg traffic model uses the following rules:
 * Each vehicle has velocity v
 * The simulation has maximum velocity v_max
 * At each time-step, a vehicle with velocity v may randomly brake (reduce v by 1) with probability p
 * At each time-step, vehicles (if they did not brake) increase speed by 1, if they are below v_max
 * At each time-step, vehicles travel forwards by v cells, or as far as they can without colliding with the next vehicle
 
This extension gives each vehicle an additional integer state value representing the movement (origin/destination pair) ranging from 1 to 12. The simulation can be viewed as a combination of normal Nagel-Schreckenberg single lanes and areas with additional rules. 

The areas with additional rules are the center of the intersection and a region extending out from the intersection by v_max cells. 

The additional rules are:
 * Each vehicle "projects priority" forwards in their path by v cells
 * If a cell is occupied by priority projection, it is treated the same as if it were occupied by a vehicle for the braking rule
 * A vehicle may not enter the intersection if it is occupied to the cell left of the entry-cell
 * A vehicle may not enter the intersection if the light is red
 * Vehicles follow paths governed by their movement index
 
These additional rules allow investigation of the effect of turn-ratios (the fraction of vehicles turning right, left, or proceeding straight) on the flow rate and traffic density at the intersection. 

The file `Intersim.m` allows adjustment of many parameters so that the user may simulate the intersection under any condition. 