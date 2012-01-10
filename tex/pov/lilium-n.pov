#include "lilium-ncommon.inc" // declare r1,r2, camera, lights, surroundings

#include "lilium-n/lnvertices.inc" // declare NV,NF, V

union {
	#macro triang(a,b,c) Etriang(a,b,c) #end
	union {
	#declare i=0; #while (i<NV) sphere {V[i] r1} #declare i=i+1; #end
	#include "lilium-n/lnfaces.inc" texture {weiss}}

	#macro triang(a,b,c) Ftriang(a,b,c) #end
	mesh { #include "lilium-n/lnfaces.inc" texture {gelb}}

rotate -x*90
}


