# PRM-with-A-Search-

************************************************************ PRM *********************************************************************

The code illustrates the PRM method to generate a roadmap for obstacle avoidance (taken circles here). The input file for the code is a CSV
that contains information about the centres of the circular obstacles and their diameters. The basic methodology of PRM is that a certain number of 
random points are sampled from the C-free space and then interconnected by edges if the said edges do not collide with the obstacles. 
To this end, to avoid repeated code, several functions were defined to expediate the coding process and make it easier for end users to 
comprehend. 

The circles are modelled as expressions of the form (x-a)^2 + (y-b)^2 - r^2. This expressions would later be used to find the intersection points
of prospective edges with the obstacles. 

List of functions :

1) plotter : To plot the sampled points, the obstacles and the edges if they are created. 

2) cfree_check : To check whether a particular sampled point lies inside the obstacles or inside the C-free space. If it is the former, the 
function flips the value of the flag variable to 1 and returns it. A for-loop iterates over all the sampled points. For each point this 
function is used, and if it returns 1, the said point is deleted from the array of points. 

3) NN4 : This function returns the 4 nearest neighbours of a particular based on the Eulerian distance. Later the function was made flexible so
that the number of neighbours could be altered. However, the NN4 name was not altered. The function takes in input as the points array and the 
index of a point in the array whose neighbours are to be found out. 

4) Line : This function takes in input as two points and returns the coefficients of the equation of straight line passing through the points. 
The equation considered is of the form : a*x + b*y + c = 0

5) intersection_points : This function takes input as the a, b and c coefficients from the Line function and the list of obstacle expressions defined
earlier. If the distance between the line and the centre of a particular circle is found to be lesser than the radius, the intersection points between that line
and that circle are found out using the sympy library and stored inside a list which is returned. 

6) location_check : This function takes input as two points coordinates and the list of intersection points returned from the intersection_points function. 
If any of the intersection points lie in between the two points, the function returns a value 0, otherwise 1. 


Basic flow of the program :

1) The obstacle information is extracted from the CSV file and stored in an array. 
2) Obstacles defined by equations which also consider a 10% padding around the obstacles. 
3) Points are sampled randomly in the area defined by -0.5<x<0.5 and -0.5<y<0.5.  The number of sampled points can be defined. 
4) Then these points are filtered using cfree_check, and only those in C-free are retained. 
5) All the required functions are defined
6) An empty edges list created. 
7) Each point in the points list is iterated through:
	i) Closest neighbours found using NN4
	ii) Iterating through each neighbour, line equation found using Line. Then, intersection points of the line with circles found using intersection_points and stored. 
		a)Each of the intersection points ( if they exist) are checked using location_check. If location_check returns a 0, that particular neighbour is abandoned. 
		b) If location_check returns 1, the edge is stored inside the edges list with the indices of the point and the neighbour and the edge length. 
8) The edges and the nodes are saved to CSV files. 


