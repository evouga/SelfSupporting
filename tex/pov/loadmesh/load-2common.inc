#include "colors.inc"
#include "farben.inc"
#include "mycolors.inc"
#include "rand.inc"

camera {
	//orthographic
	location <-1,0,.3>*35 rotate z*(58)
		translate <5,5,0>
	angle 40
	look_at <0,0,0>
	sky <0,0,1>
}

//sphere {<5,10,0> 1}

light_source { <-10,-20,30>*25 color rgb <1,1,1> *.6 
	//shadowless
	area_light <20,0,0>*10 <0,20,0>*10 5 5 jitter circular 
	}
light_source { <-10,-10,20>*25 color rgb <1,1,1> *0.7 shadowless}
light_source { <0,0,-20>*25 color rgb <1,1,1> *0.8 shadowless}


//plane {z,-1 texture {gelb}}
background {color rgb <1,1,1>}
box {<-15,-15,-2> <10,10,-1> texture {weiss}}
#declare r1=0.002;
#declare r2=0.002;

#include "facemacros.inc"

#macro Mesh(A) union { mesh { #include A texture {weisst}}
	mesh { #include A translate y*.001 texture {brownish}}
	rotate -x*90 } #end

