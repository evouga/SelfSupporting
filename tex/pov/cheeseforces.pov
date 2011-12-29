#include "cheese-common.inc" // declare r1,r2, camera, lights, surroundings

#include "cheese/cheese-nvertices.inc" // declare NV,NF, V

#macro farbe(i)
        //pigment {color rgbt (1-i/4)*<.15,.2,.3,0.2>+(i/4)*<.7,.5,.2,.2>}
        pigment {color rgbt (1-i/4)*<.7,.7,.2,0>+(i/4)*<1,1,1,0>*.7}
        finish {ambient 0.7 diffuse .5}
#end


union {
	//#macro Quad(a,b,c,d) EQuad(a,b,c,d) #end
	//union {
	//#declare i=0; #while (i<NV) sphere {V[i] r1} #declare i=i+1; #end
	//#include "cheese/cheesefaces.inc" texture {brownish}}
	#macro WEdge(i,j,w) 
		//cylinder{V[i] V[j] w*r1*90*vlength(V[i]-V[j])} 
		cylinder{V[i] V[j] pow(w*r1*vlength(V[i]-V[j]),.5)*.8} 
		#end
	union {#include "cheese/cheeseweights.inc" texture {brownish}}

	#macro Quad(a,b,c,d) FQuad(a,b,c,d) #end
	mesh { #include "cheese/cheesefaces.inc" texture {weisst}}

rotate -x*90
}

