#include "colors.inc"
#include "farben.inc"
#include "mycolors.inc"
#include "rand.inc"

#declare M = 6;
#declare N = 6;
#declare factor=2.0/(M+N);

camera {
	orthographic
	location <-1,0,.8>*22*factor rotate z*20
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

#include "triangmacros1.inc"


declarepoints(0.18)
declarecircums()

plane {z,-0.1*factor texture {weiss} }
//box {<-1.1,-1.1,-0.3*factor><1.1,1.1,-0.2*factor> rotate z*10 texture {gelb}}
background{White}

union {
	union {showpoints(pkte,M,N,r1) texture {weiss}}
	union { showedges(pkte,M,N,r1) texture {gelb}}
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


/*
// paraboloid
mesh {
#declare r0=0; #declare dr=0.1;
#while (r0<1)
	#declare ang0=0; #declare dang=3.1415926/20;
	#while (ang0 < 2*3.1415926)		
		#declare r1=r0+dr;
		#declare ang1=ang0+dang;
		#declare P0 = <r0*cos(ang0),r0*sin(ang0),r0*r0/2>;
		#declare P1 = <r0*cos(ang1),r0*sin(ang1),r0*r0/2>;
		#declare P2 = <r1*cos(ang0),r1*sin(ang0),r1*r1/2>;
		#declare P3 = <r1*cos(ang1),r1*sin(ang1),r1*r1/2>;

		#declare N0 = vnormalize(<P0.x,P0.y,-1>);
		#declare N1 = vnormalize(<P1.x,P1.y,-1>);
		#declare N2 = vnormalize(<P2.x,P2.y,-1>);
		#declare N3 = vnormalize(<P3.x,P3.y,-1>);

		//triangle{P0,P1,P2}
		//triangle{P2,P1,P3}
		smooth_triangle{P0,N0,P1,N1,P2,N2}
		smooth_triangle{P2,N2,P1,N1,P3,N3}

		#declare ang0=ang1;
	#end	
	#declare r0=r1;
#end
	translate vertical+Left
	texture {pigment {color rgbt <1,1,1,.2>}}
}
*/
