#include "remove/remove-common.inc" // declare r1,r2, camera, lights, surroundings

#include "remove/remove-vertices.inc" // declare NV,NF, V

#macro triang(a,b,c) Ftriang(a,b,c) #end
Mesh ("remove/remove-faces.inc")


