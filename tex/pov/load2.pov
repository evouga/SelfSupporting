#include "loadmesh/load-2common.inc" // declare r1,r2, camera, lights, surroundings

#include "loadmesh/load2vertices.inc" // declare NV,NF, V

#macro triang(a,b,c) Ftriang(a,b,c) #end
Mesh("loadmesh/load2faces.inc")

box {
	#declare ff=1.5;
	<-.82/ff,0,-.82/ff>
	<.82/ff,-1.64*ff*ff,.82/ff>
	translate V[1]
	rotate -x*90
	texture {brownish}
}

