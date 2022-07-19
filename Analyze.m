%CA Flow Analysis
%Alexander Farley
%alexander.farley at utoronto.ca
%April 26 2011
%------------------------------------------------------------------------------
%This script is used to analyze flow and density in a CA traffic simulation
%recording


if 1 %Plot all occupation samples
figure
subplot(1,4,1)
imshow(1-southbound_occupation)
%title('North-South')
%xlabel('Position')
%ylabel('Timesteps')
subplot(1,4,2)
imshow(1-northbound_occupation)
%title('South-North')
%xlabel('Position')
%ylabel('Timesteps')
subplot(1,4,3)
imshow(1-westbound_occupation)
%title('East-West')
%xlabel('Position')
%ylabel('Timesteps')
subplot(1,4,4)
imshow(1-eastbound_occupation)
%title('West-East')
%xlabel('Position')
%ylabel('Timesteps')
end

if 0 %Plot all velocity samples
figure
subplot(1,4,1)
imshow(southbound_velocity)
title('North-South v')
subplot(1,4,2)
imshow(northbound_velocity)
title('South-North v')
subplot(1,4,3)
imshow(westbound_velocity)
title('East-West v')
subplot(1,4,4)
imshow(eastbound_velocity)
title('West-East v')
end

if 0
figure
imshow(southbound_occupation)
title('Southbound occupation')
end

if 0 %Northbound flow vs density
    figure
    scatter(north_flow_variables(:,2), north_flow_variables(:,1));
end

if 0 %Total flow vs density
    total_flow = (north_flow_variables(:,1) + south_flow_variables(:,1) + east_flow_variables(:,1) + west_flow_variables(:,1))/4;
    total_density = (north_flow_variables(:,2) + south_flow_variables(:,2) + east_flow_variables(:,2) + west_flow_variables(:,2))/4;
    figure
    scatter(total_density, total_flow);
    title('Flow vs density')
    xlabel('Density (vehicles/cell)')
    ylabel('Flow (vehicle-cells/timestep)')
end

if 0 %Northbound flow vs left-turn proportion
     figure
     scatter(north_flow_variables(:,3), north_flow_variables(:,1));
     title('Northbound flow vs left-turn proportion')
     xlabel('Left turn proportion')
     ylabel('Flow (vehicle-cells per timestep)')
     axis([0 1 0 1])
end

if 0 %Northbound flow vs right-turn proportion
     figure
     scatter(north_flow_variables(:,5), north_flow_variables(:,1));
     title('Northbound flow vs right-turn proportion')
     xlabel('Right turn proportion')
     ylabel('Flow (vehicle-cells per timestep)')
     axis([0 1 0 1])
end

if 0 %Northbound flow vs straight-through proportion
    figure
     scatter(north_flow_variables(:,4), north_flow_variables(:,1));
     title('Northbound flow vs straight-through proportion')
     xlabel('Straight-through proportion')
     ylabel('Flow (vehicle-cells per timestep)')
     axis([0 1 0 1])
end

if 0 %Plot flow vs LR plane
    figure
    im_normalized = ContrastStretchNorm(flow_samples);
    imshow(im_normalized);
    xlabel('Right turn proportion')
    ylabel('Left turn proportion')
end

error_count

