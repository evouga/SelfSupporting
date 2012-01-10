#include "colors.inc"
#include "farben.inc"
#include "mycolors.inc"
#include "rand.inc"

camera {
	//orthographic
	location <-1,0,.1>*35 
		rotate z*(50)
		translate <8,10,3>
	angle 50
	look_at <0,0,3>
	sky <0,0,1>
}

//sphere {<8,12,3>,2}


light_source { <-10,-20,30>*100 color rgb <1,1,1> *.6 
	//shadowless
	area_light <20,0,0>*100 <0,20,0>*100 5 5 jitter circular 
	}
light_source { <-10,-10,20>*25 color rgb <1,1,1> *0.7 shadowless}
light_source { <0,0,-20>*25 color rgb <1,1,1> *0.8 shadowless}


//plane {z,-0.1 texture {weiss}}
background {color rgb <1,1,1>}
box {<-4,-3,-3>*3 <25,25,0> texture {weiss}}
#declare r1=0.02;
#declare r2=0.02;

#include "facemacros.inc"
