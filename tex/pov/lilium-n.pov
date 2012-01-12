#include "lilium-ncommon.inc" // declare r1,r2, camera, lights, surroundings

#include "lilium-n/lnvertices.inc" // declare NV,NF, V

union {
	#macro triang(a,b,c) Etriang(a,b,c) #end
	union { #include "lilium-n/lnfaces.inc" texture {weiss}}
rotate -x*90
}

SMesh( "lilium-n/lnfaces.inc", gelb)



