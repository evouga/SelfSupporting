#include "loadmesh/load-common.inc" // declare r1,r2, camera, lights, surroundings

#include "loadmesh/load-nvertices.inc" // declare NV,NF, V

#macro triang(a,b,c) Ftriang(a,b,c) #end
Mesh("loadmesh/loadmeshfaces.inc")

box {
	#declare ff=2.0;
	<.1*ff,0,.1*ff>, -<.1*ff, 6.5/(ff*ff), .1*ff>
	translate V[12]
	rotate -x*90
}

