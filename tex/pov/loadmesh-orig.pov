#include "loadmesh/load-common.inc" // declare r1,r2, camera, lights, surroundings

#include "loadmesh/load-vertices.inc" // declare NV,NF, V

union {
	#macro triang(a,b,c) Etriang(a,b,c) #end
	union {
	#declare i=0; #while (i<NV) sphere {V[i] r1} #declare i=i+1; #end
	#include "loadmesh/loadmeshfaces.inc" texture {weiss}}

	#macro triang(a,b,c) Ftriang(a,b,c) #end
	mesh { #include "loadmesh/loadmeshfaces.inc" texture {gelb}}

rotate -x*90
}

