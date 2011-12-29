#include "cas/cas-common.inc" // declare r1,r2, camera, lights, surroundings

#include "cas/cas-nvertices.inc" // declare NV,NF, V

union {
	#macro Quad(a,b,c,d) EQuad(a,b,c,d) #end
	union {
	#declare i=0; #while (i<NV) sphere {V[i] r1} #declare i=i+1; #end
	#include "cas/casfaces.inc" texture {weiss}}

	#macro Quad(a,b,c,d) FQuad(a,b,c,d) #end
	mesh { #include "cas/casfaces.inc" texture {gelb}}

rotate -x*90
}

