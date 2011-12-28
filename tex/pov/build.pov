#include "build-common.inc" // declare r1,r2, camera, lights, surroundings

#include "build/b-vertices.inc" // declare NV,NF, V

#macro farbe(i)
        //pigment {color rgbt (1-i/4)*<.15,.2,.3,0.2>+(i/4)*<.7,.5,.2,.2>}
        pigment {color rgbt (1-i/4)*<.7,.7,.2,0>+(i/4)*<1,1,1,0>*.7}
        finish {ambient 0.7 diffuse .5}
#end


union {
	#macro Quad(a,b,c,d) EQuad(a,b,c,d) #end
	union {
	#declare i=0; #while (i<NV) sphere {V[i] r1} #declare i=i+1; #end
	#include "build/b-faces.inc" texture {brownish}}

	#macro Quad(a,b,c,d) FQuad(a,b,c,d) #end
	mesh { #include "build/b-faces.inc" texture {weiss} // {farbe(1)}
		}

rotate -x*90
}



