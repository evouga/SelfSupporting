#include "load/load-common.inc" // declare r1,r2, camera, lights, surroundings

#include "load/dome2-00-vertices.inc" // declare NV,NF, V
#include "load/dome2-00-normals.inc" // declare N

#macro triang(a,b,c) FStriang(a,b,c) #end
SMesh("load/dome2-faces.inc", concrete)

union { union {#include "load/dome2-00-weights.inc" texture {dblau}}
	rotate -x*90 }


