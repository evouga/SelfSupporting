#include "colors.inc"
#include "farben.inc"
#include "mycolors.inc"
#include "rand.inc"

camera {
	orthographic
	location <-1,0,-.7>*5 rotate z*10
	angle 30
	look_at <0,0,-0.5>
	sky <0,0,1>
}


light_source { <-10,-20,30> color rgb <1,1,1> *.6 
	//shadowless
	//area_light <20,0,0> <0,20,0> 5 5 jitter circular 
	}
light_source { <0,0,-20> color rgb <1,1,1> *0.7 shadowless}
light_source { <0,0,-20> color rgb <1,1,1> *0.7 shadowless}


//plane {z,-0.9 texture {weiss}}
background{White}
#declare r1=0.002;
#declare r2=0.002;

#include "facemacros.inc"

#include "lilium-pillar-n/lpnvertices.inc" // declare NV,NF, V

union {
	#macro triang(a,b,c) Etriang(a,b,c) #end
	union {
	#declare i=0; #while (i<NV) sphere {V[i] r1} #declare i=i+1; #end
	#include "lilium-pillar-n/lpnfaces.inc" texture {weiss}}

	#macro triang(a,b,c) Ftriang(a,b,c) #end
	mesh { #include "lilium-pillar-n/lpnfaces.inc" texture {gelb}}

rotate -x*90
}



