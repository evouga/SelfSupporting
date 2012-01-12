#include "colors.inc"
#include "farben.inc"
#include "mycolors.inc"
#include "rand.inc"

camera {
	//orthographic
	location <-1,0,.4>*35 rotate z*(38)
		translate <5,12,0>
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


//plane {z,-0.1 texture {weiss}}
background {color rgb <1,1,1>}
box {<-2,-2,-.01>*10 <8,12,0>*2.5 texture {weiss}}
#declare r1=0.002;
#declare r2=0.002;


#include "facemacros.inc"

#macro Mesh(A) DMesh(A, weisst, brownish) #end

#macro Pfeil(I) union {
	sphere { V[I] 0.2 texture {gelb}}
	//cylinder {V[I]-2*y V[I]-3*y 0.1 texture {gelb}}
	//cone {V[I]-1*y 0 V[I]-2*y 0.3 texture {gelb}}
	rotate -x*90} 
	#end
