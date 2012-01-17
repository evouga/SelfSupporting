#include "colors.inc"
#include "farben.inc"
#include "mycolors.inc"
#include "rand.inc"

camera {
	orthographic
	location <-1,0,.7>*15 
	angle 30
	look_at <0,0,-0.5>
	sky <0,0,1>
}


light_source { <-10,-20,30>*2 color rgb <1,1,1> *.6 
	//shadowless
	area_light <20,0,0>*5 <0,20,0>*5 5 5 jitter circular 
	}
light_source { <0,0,20> color rgb <1,1,1> *0.7 shadowless}
light_source { <0,0,-20> color rgb <1,1,1> *0.7 shadowless}


plane {z,-.8 texture {weiss}}
#declare r1=0.003;
#declare r2=0.003;

#include "facemacros.inc"
