#include "remove/remove-common.inc" // declare r1,r2, camera, lights, surroundings

#include "remove/remove-4nvertices.inc" // declare NV,NF, V

#macro triang(a,b,c) Ftriang(a,b,c) #end
Mesh ("remove/remove-4nfaces.inc")

