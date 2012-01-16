#include "colors.inc"
#include "farben.inc"
#include "mycolors.inc"
#include "rand.inc"

camera {
	orthographic
	location <-1,0,1.0>*5 rotate z*10
	angle 65
	look_at <0,0,1.2>
	sky <0,0,1>
}


//light_source { <-10,-20,30> color rgb <1,1,1> *.6 
light_source { <-15,-5,12>*2 color rgb <1,1,1> *1.1 
	//shadowless
	area_light <20,0,0> <0,20,0> 5 5 circular jitter 
	}   
light_source { <0,0,20> color rgb <1,1,1> *0.7 shadowless}
light_source { <0,0,-20> color rgb <1,1,1> *0.7 shadowless}


//plane {z,-.8 texture {weiss}}
plane {x,+1.5 texture {weiss}}
#declare r1=0.002;
#declare r2=0.002;

#include "facemacros.inc"
