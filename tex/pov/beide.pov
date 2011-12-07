#include "colors.inc"
#include "farben.inc"
#include "mycolors.inc"
#include "rand.inc"

#declare M = 8;
#declare N = 7;
#declare factor=2.5/(M+N);

camera {
	orthographic
	location <-1,0,0.5>*22*factor rotate z*20
	angle 55
	look_at <0,0,0.5>
	sky <0,0,1>
}


light_source { <-10,-20,30> color rgb <1,1,1> *.6 
	//shadowless
	area_light <20,0,0> <0,20,0> 5 5 jitter circular 
	}
light_source { <0,0,20> color rgb <1,1,1> *0.7 shadowless}
light_source { <0,0,-20> color rgb <1,1,1> *0.7 shadowless}


#declare r1=.05*factor;
#declare r2=.03*factor;
#declare vertical = 2*z*factor;

#declare Right = <-0.5,1.4,0>*0.60;
#declare Left = -Right;

#include "triangmacros.inc"


declarepoints(0.18)
declarecircums()

plane {z,-0.1*factor texture {weiss} }
//box {<-1.1,-1.1,-0.3*factor><1.1,1.1,-0.2*factor> rotate z*10 texture {gelb}}
background{White}

union {
	union {showpoints(pkte,M,N,r1) texture {weiss}}
	union { showedges(pkte,M,N,r2) texture {gelb}}
	translate Right
}

union { 
	union {showdualedges(circums1,circums2,M,N,r2) texture {dblau}}
	translate Left
}

// points on the Maxwell paraboloid

movepointstomaxwell(pkte)
movedualpointstomaxwell(circums1,circums2)


union {
	union {showpoints(pkte,M,N,r2) texture {gelb}}
	union { showedges(pkte,M,N,r2) texture {gelb}}
	mesh { showfaces(pkte,M,N) texture {brownish}}
	translate vertical
	translate Right
}

union {
	union { showdualedges(circums1,circums2,M,N,r2) texture {blau}}
	mesh { showdualfaces(circums1,circums2,M,N) texture {grau_schwein}}
	translate vertical
	translate Left
}

