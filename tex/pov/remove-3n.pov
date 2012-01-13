#include "remove/remove-common.inc" // declare r1,r2, camera, lights, surroundings

#include "remove/remove-3nvertices.inc" // declare NV,NF, V
#include "remove/remove-3nnormals.inc" // declare N

#macro triang(a,b,c) FStriang(a,b,c) #end
SMesh("remove/remove-3nfaces.inc", concrete)

Pfeil(288)
