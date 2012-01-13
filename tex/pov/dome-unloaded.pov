#include "load/load-common.inc" // declare r1,r2, camera, lights, surroundings

#include "load/dome-vertices.inc" // declare NV,NF, V
#include "load/dome-normals.inc" // declare N

#macro triang(a,b,c) FStriang(a,b,c) #end
SMesh("load/dome-faces.inc", concrete)

union { union {#include "load/dome-unloaded-weights.inc" texture {dblau}}
	rotate -x*90 }


