function [cell_visit_map inc_coverage_rate new_cells_covered] = cellmap(xpoints,ypoints,px,py,ns,dt,cell_visit_map)
%Written by Karan Sridharan
%Calculates the new tracking position of the Robot's trajectory

%Input variables:
%xpoints: points located in the x-axis
%ypoints: points located in the y-axis
%px: vector, p containing the x coordinates of the Robot's trajectory
%py:  p containing the y coordinates of the Robot's trajectory
%ns: Number of rows and columns of the cell_visit_map
%cell_visit_map: Initial tracking position of the Robot's trajectory

%Output variables:
%cell_visit_map: New tracking position of the Robot's trajectory
%inc_coverage_rate: Coverage rate that denotes how many new cells are being
%covered in a second
new_cells_covered = 0;  
for i = 1:length(ypoints)-1
    for j = 1:length(xpoints)-1
         number_cells_visited = 1; 
         %Number of cells visited. Presets to one

            %k is an index value that gets the index number of points that are lying
            %within the boundares of a particular cell. The purpose of this
            %step is to identify whether each point occurs at the cell at
            %instantaneous time or at a later time
            k = find(px>=xpoints(j)&px<=xpoints(j+1)&py>=ypoints(i)&py<=ypoints(i+1));
            
            if length(k) ~=0 
                 if cell_visit_map(i,j) ~= 0 %This code will execute if a cell has been covered in the last iteration
                     if length(k) ~= 1
                         for l = 1:length(k)-1 
                             if k(l+1)-k(l) ~= 1; %If the robot visits and does not visit the cell at a later time, number of cells being covered increments by one
                                 number_cells_visited = number_cells_visited + 1;
                             end
                         end
                         
                         cell_visit_map(i,j) = cell_visit_map(i,j) + number_cells_visited;
                     end
                     
                     if length(k) == 1 %This line calculates the number of cells covered if k finds there is one point that comes between the selected range
                         cell_visit_map(i,j) = cell_visit_map(i,j) + number_cells_visited;
                     end
                         
                 else %This line will execute if a cell has not been covered in the last iteration
                     
                     if length(k) ~= 1
                         for l = 1:length(k)-1 
                        %What this line means is say for instance at index
                        %numbers 86 and 96, the points are lying between x =
                        %-0.25 and -0.25 and y = -2.5 and -2.45. At the 96th
                        %index, the Robot has returned to the same cell at a
                        %later time. Therefore, this condition says that if the
                        %Robot has returned to the same cell at a later time,
                        %add that to the number of times the Robot has visited
                        %that cell
                        if k(l+1)-k(l) ~= 1;
                            number_cells_visited = number_cells_visited + 1;
                        end
                         end
                      cell_visit_map(i,j) = number_cells_visited; 
                      new_cells_covered = new_cells_covered + 1;
                     
                     end
                     %Suppose there are only one point that lie in a particular cell, the the
                     %cell map will register that cell as been covered once
                     if length(k) == 1
                         new_cells_covered = new_cells_covered + 1;
                         cell_visit_map(i,j) = number_cells_visited;
                     end
                 end
            end
    end
end
inc_coverage_rate = new_cells_covered/(100*dt*ns^2);