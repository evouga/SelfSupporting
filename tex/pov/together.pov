#include "colors.inc"
#include "farben.inc"
#include "mycolors.inc"
#include "rand.inc"

#declare M = 9;
#declare N = 7;
#declare factor=2.0/(M+N);

camera {
	orthographic
	location <-1,0,.8>*22*factor rotate z*25
	angle 35
	look_at <0,0,0>
	sky <0,0,1>
}


light_source { <-10,-20,30> color rgb <1,1,1> *.6 
	//shadowless
	//area_light <20,0,0> <0,20,0> 5 5 jitter circular 
	}
light_source { <0,0,20> color rgb <1,1,1> *0.7 shadowless}
light_source { <0,0,-20> color rgb <1,1,1> *0.7 shadowless}


#declare r1=.08*factor;
#declare r2=.03*factor;
#declare vertical = 2*z*factor;


#include "triangmacros.inc"


declarepoints(0.18)
declarecircums()

//plane {z,-0.1*factor texture {weiss} }
box {<-1.1,-1.1,-0.3*factor><1.1,1.1,-2*r1> rotate z*10 texture {weiss}}
background{White}

#declare i0=1;
#declare j0=1;


union {
        union {showpoints(pkte,M,N,r1) texture {weiss}}
        union { showedges(pkte,M,N,r2) texture {gelb}}
}

union { 
	union {showdualedges(circums1,circums2,M,N,r2) texture {dblau}}
	union { showpoints(circums1,M-1,N-1,r1) texture {weiss}}

}

