#include "build/build-common.inc" // declare r1,r2, camera, lights, surroundings

#include "build/ben-vertices.inc" // declare NV,NF, V

union {
	#macro Quad(a,b,c,d) EQuad(a,b,c,d) #end
	union {
	#declare i=0; #while (i<NV) sphere {V[i] r1} #declare i=i+1; #end
	#include "build/be-faces.inc" texture {weiss}}

	#macro Quad(a,b,c,d) FQuad(a,b,c,d) #end
	mesh { #include "build/be-faces.inc" texture {gelb}}

rotate -x*90
}



