#include "lilium-ncommon.inc" // declare r1,r2, camera, lights, surroundings

#include "lilium-n/ln-stressvertices.inc" // declare NV,NF, V

#declare r1=r1*2/3;
#declare r2=r1;

union {
	#macro triang(a,b,c) Etriang(a,b,c) #end
	union {
	//#declare i=0; #while (i<NV) sphere {V[i] r1} #declare i=i+1; #end
	#include "lilium-n/ln-stressfaces.inc" texture {dblau}}

	#macro triang(a,b,c) Ftriang(a,b,c) #end
	mesh { #include "lilium-n/ln-stressfaces.inc" 
	//texture {pigment {color rgbt <1,1,1,.3>} finish{ambient 0.4} } }
	texture {weiss}}
	scale -y*2
rotate -x*90
	translate -z*0.4
}



#include "lilium-n/ln-rvertices.inc" // declare NV,NF, V

union {
	#macro Quad(a,b,c,d) EQuad(a,b,c,d) #end
	#macro Hex(a,b,c,d,e,f) EHex(a,b,c,d,e,f) #end
	union {
	//#declare i=0; #while (i<NV) sphere {V[i] r1} #declare i=i+1; #end
	#include "lilium-n/ln-rfaces.inc" texture {brownish}}

	#macro Quad(a,b,c,d) FQuad(a,b,c,d) #end
	#macro Hex(a,b,c,d,e,f) FHex(a,b,c,d,e,f) #end
	mesh { #include "lilium-n/ln-rfaces.inc" texture {weiss}}
	translate -<0.2,0,-0.3> // near center of reciprocal diagr.
	scale 2.5
rotate -x*90
	translate -z*0.8 // near plane
}



