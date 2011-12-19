#include "lilium-ncommon.inc" // declare r1,r2, camera, lights, surroundings

#include "lilium-n/ln-rvertices.inc" // declare NV,NF, V

#declare r1=r1/2;
#declare r2=r1;

//plane {z,-0.1 texture {weiss}}

union {
	#macro Quad(a,b,c,d) EQuad(a,b,c,d) #end
	#macro Hex(a,b,c,d,e,f) EHex(a,b,c,d,e,f) #end
	union {
	//#declare i=0; #while (i<NV) sphere {V[i] r1} #declare i=i+1; #end
	#include "lilium-n/ln-rfaces.inc" texture {dblau}}

	#macro Quad(a,b,c,d) FQuad(a,b,c,d) #end
	#macro Hex(a,b,c,d,e,f) FHex(a,b,c,d,e,f) #end
	mesh { #include "lilium-n/ln-rfaces.inc" texture {weiss}}
	translate -<0.1,0,-0.22> // near center of reciprocal diagr.
	scale 3
rotate -x*90
	translate -z*0.8 // near plane
}



