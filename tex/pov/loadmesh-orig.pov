#include "loadmesh/load-common.inc" // declare r1,r2, camera, lights, surroundings

#include "loadmesh/load-vertices.inc" // declare NV,NF, V

#macro triang(a,b,c) Ftriang(a,b,c) #end
Mesh("loadmesh/loadmeshfaces.inc")

