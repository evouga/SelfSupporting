#include "colors.inc"
#include "farben.inc"
#include "mycolors.inc"
#include "rand.inc"

#declare M = 6;
#declare N = 6;
#declare factor=2.0/(M+N);

camera {
	orthographic
	location <-1,0,.8>*22*factor rotate z*25
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


#declare r1=.08*factor;
#declare r2=.03*factor;
#declare vertical = 2*z*factor;

#declare Right = <-0.5,1.4,0>*0.60;
#declare Left = -Right;

#include "triangmacros.pov"


declarepoints(0.18)
declarecircums()

plane {z,-0.1*factor texture {weiss} }
//box {<-1.1,-1.1,-0.3*factor><1.1,1.1,-0.2*factor> rotate z*10 texture {gelb}}
background{White}

#declare i0=1;
#declare j0=1;

// three dual edges emanating from s (in paper: primal!)
#declare Sp=circums1[i0+1][j0+1]; 
#declare B1=circums2[i0+1][j0+1]; // S-B1 dual to P-Q
#declare B2=circums2[i0+1][j0+0]; // S-B2 dual to Q-R
#declare B3=circums2[i0+2][j0+1]; // S-B3 dual to R-P
#declare V1=vnormalize(B1-Sp); 
#declare V2=vnormalize(B2-Sp);
#declare V3=vnormalize(B3-Sp);

// corresponding primal edges (in paper: dual!)
#declare P=pkte[i0+2][j0+2];
#declare Q=pkte[i0+1][j0+1];
#declare R=pkte[i0+2][j0+1];
#declare L1=vlength(P-Q);
#declare L2=vlength(Q-R);
#declare L3=vlength(R-P);
// dual edges 

union {
	union {showpoints(pkte,M,N,r1) texture {weiss}}
	union { showedges(pkte,M,N,r2) texture {gelb}}
	union {
		cylinder {(P+Q)/2, P,r1} cone {Q,0,(P+Q)/2,2*r1}
		cylinder {(Q+R)/2, Q,r1} cone {R,0,(Q+R)/2,2*r1}
		cylinder {(R+P)/2, R,r1} cone {P,0,(R+P)/2,2*r1}
		texture {rot}
	}
	translate Right
}


union { 
	union {showdualedges(circums1,circums2,M,N,r2) texture {dblau}}
	union { showpoints(circums1,M-1,N-1,r1) texture {weiss}}

	union {
		cylinder {Sp+V1*L1,Sp+V1*L1/2,r1}
		cylinder {Sp+V2*L2,Sp+V2*L2/2,r1}
		cylinder {Sp+V3*L3,Sp+V3*L3/2,r1}
		cone {Sp,0,Sp+V1*L1/2,2*r1}
		cone {Sp,0,Sp+V2*L2/2,2*r1}
		cone {Sp,0,Sp+V3*L3/2,2*r1}
		texture {rot}
	}
	translate Left
}

// points on the Maxwell paraboloid

movepointstomaxwell(pkte)
movedualpointstomaxwell(circums1,circums2)

// prepare to draw arrows
#declare ol1=vlength(B1-Sp); // store original edgelengths
#declare ol2=vlength(B2-Sp);
#declare ol3=vlength(B3-Sp);
#declare Sp=circums1[i0+1][j0+1];  // re-defined
#declare B1=circums2[i0+1][j0+1]; // re-defined S-B1 dual to P-Q
#declare B2=circums2[i0+1][j0+0]; // re-defined S-B2 dual to Q-R
#declare B3=circums2[i0+2][j0+1]; // re-defined S-B3 dual to R-P
#declare V1=vnormalize(B1-Sp); // re-defined
#declare V2=vnormalize(B2-Sp);// re-defined
#declare V3=vnormalize(B3-Sp);// re-defined
// new iedgelengths
#declare L1=L1*vlength(B1-Sp)/ol1; // redefine lengths of arrows
#declare L2=L2*vlength(B2-Sp)/ol2;
#declare L3=L3*vlength(B3-Sp)/ol3;
// force which is in balance
#declare FF = V1*L1+V2*L2+V3*L3;

// corresponding primal edges (in paper: dual!)
#declare P=pkte[i0+2][j0+2];


union {
	union { showpoints(circums1,M-1,N-1,r1) texture {weiss}}
	union { showdualedges(circums1,circums2,M,N,r2) texture {blau}}
	mesh { showdualfaces(circums1,circums2,M,N) 
		texture {pigment {color rgbt <.6,.6,.62,.4>} 
		finish {ambient 0.2 diffuse 0.8 phong 0.2}}}
	union {
		cylinder {Sp+V1*L1,Sp+V1*L1/2,r1}
		cylinder {Sp+V2*L2,Sp+V2*L2/2,r1}
		cylinder {Sp+V3*L3,Sp+V3*L3/2,r1}
		cylinder {Sp+FF/2,Sp,r1}
		cone {Sp,0,Sp+V1*L1/2,2*r1}
		cone {Sp,0,Sp+V2*L2/2,2*r1}
		cone {Sp,0,Sp+V3*L3/2,2*r1}
		cone {Sp+FF,0,Sp+FF/2,2*r1}
		texture {dblau}
	}
	scale z*(-0.7)
	translate 3*vertical
	translate Left
}


