#include "lilium-ncommon.inc" // declare r1,r2, camera, lights, surroundings


#macro farbe(i)
        //pigment {color rgbt (1-i/4)*<.15,.2,.3,0.2>+(i/4)*<.7,.5,.2,.2>}
        pigment {color rgbt (1-i/4)*<.7,.7,.2,0>+(i/4)*<1,1,1,0>*.7}
        finish {ambient 0.7 diffuse .5}
#end

#include "lilium-n/lvertices.inc" // declare NV,NF, V

union {
	#macro triang(a,b,c) Etriang(a,b,c) #end
	union {
	#declare i=0; #while (i<NV) sphere {V[i] r1} #declare i=i+1; #end
	#include "lilium-n/lfaces.inc" texture {weiss}}

	#macro triang(a,b,c) Ftriang(a,b,c) #end
	mesh { #include "lilium-n/lfaces.inc" texture {farbe(1)}}

rotate -x*90
}


