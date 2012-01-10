#include "loadmesh/load-common.inc" // declare r1,r2, camera, lights, surroundings

#include "loadmesh/load-nvertices.inc" // declare NV,NF, V

union {
	#macro triang(a,b,c) Etriang(a,b,c) #end
	union {
	#declare i=0; #while (i<NV) sphere {V[i] r1} #declare i=i+1; #end
	#include "loadmesh/loadmeshfaces.inc" texture {brownish}}

	#macro triang(a,b,c) Ftriang(a,b,c) #end
	mesh { #include "loadmesh/loadmeshfaces.inc" texture {weiss}}

	box {
		#declare ff=2.0;
		<.1*ff,0,.1*ff>, -<.1*ff, 6.5/(ff*ff), .1*ff>
		translate V[12]
	}

rotate -x*90
}
