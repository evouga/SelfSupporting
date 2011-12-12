#include "colors.inc"
#include "farben.inc"
#include "mycolors.inc"
#include "rand.inc"

camera {
	orthographic
	location <-1,0,.7>*5 rotate z*20
	angle 30
	look_at <0,0,0.5>
	sky <0,0,1>
}


light_source { <-10,-20,30> color rgb <1,1,1> *.6 
	//shadowless
	area_light <20,0,0> <0,20,0> 5 5 jitter circular 
	}
light_source { <0,0,20> color rgb <1,1,1> *0.7 shadowless}
light_source { <0,0,-20> color rgb <1,1,1> *0.7 shadowless}


#declare Right = <-0.5,1.4,0>*0.5;
#declare Left = -Right;


plane {z,-0.1 texture {weiss}}

#declare A = array[5] {
	 <.4, .4, 0>,
	<.4, -.4, 0>,
	 <-.4, -.4, 0>,
	 <-.4, .4, 0>
	 <.4, .4, 0>,
	};

#declare vv = -0.1*x-0.1*y - .5*z;
#declare B = array[5] {
	 <-.3, .3, 0>-vv,
	<-.3, -.3, 0>-vv,
	 <.3, -.3, 0>-vv,
	 <.3, .3, 0>-vv
	 <-.3, .3, 0>-vv,
	};


#macro P(i) <A[i].x, A[i].y, -1>  #end
#macro Q(i) <B[i].x, B[i].y, -1>  #end

#declare E = array[5] {
	vcross(P(0),P(1)),
	vcross(P(1),P(2)),
	vcross(P(2),P(3)),
	vcross(P(3),P(0)),
	vcross(P(0),P(1)), };
#declare F = array[5] {
	vcross(Q(0),Q(1)),
	-vcross(Q(1),Q(2)),
	vcross(Q(2),Q(3)),
	-vcross(Q(3),Q(0)),
	vcross(Q(0),Q(1)), };


#macro farbe(i) 
	//pigment {color rgbt (1-i/4)*<.15,.2,.3,0.2>+(i/4)*<.7,.5,.2,.2>} 
	pigment {color rgbt (1-i/4)*<.7,.7,.2,0>+(i/4)*<1,1,1,0>*.7}
	finish {ambient 0.7 diffuse .5}
#end

#declare rr=.5; // bounding cylinder radius
#declare r1=0.03; // size of vertices
#declare r2=0.007; // thickness of edges

union {
	 #declare i=1; #while (i<5)
	#declare EV=rr*E[i]/sqrt(E[i].x*E[i].x+E[i].y*E[i].y);
	union {
		plane {P(i) 0 texture {farbe(i)}
			clipped_by {plane {<E[i].x,E[i].y,0> 0}}
			clipped_by {plane {<E[i-1].x,E[i-1].y,0> 0}} 
			clipped_by {cylinder {-2*z,3*z,rr}}}
		cylinder {0, -EV, r2 texture {weiss}}
		sphere {-EV, r2 texture {weiss}}
		translate -A[1].z *z
	}
	#declare FV=rr*F[i]/sqrt(F[i].x*F[i].x+F[i].y*F[i].y);
	union { 
		plane {Q(i) 0 texture {farbe(i)}
			clipped_by {plane {<F[i].x,F[i].y,0> 0}}
			clipped_by {plane {<F[i-1].x,F[i-1].y,0> 0}} 
			clipped_by {cylinder {-2*z,3*z,rr}}}
		cylinder {0, -FV, r2 texture {weiss}}
		sphere {-FV, r2 texture {weiss}}
		translate -B[1].z *z
	}
	#declare i=i+1; #end
	sphere {0,r1 texture{brownish}}
	sphere {0,r1 texture{brownish} translate -B[1].z *z}
	translate Left + 0.65*z
}

union
{
	#declare i=1; #while(i<5) 
		sphere {A[i],r1 texture {farbe(i)}} 
		sphere {B[i],r1 texture {farbe(i)}} 
	#declare i=i+1; #end
	mesh {
		triangle {A[0],A[1],A[2]}
		triangle {A[0],A[2],A[3]}
		triangle {B[0],B[1],B[2]}
		triangle {B[0],B[2],B[3]}
		texture {brownish}
	}
	cylinder {A[0] A[2] r1/3 texture{weiss}}
	cylinder {A[1] A[3] r1/3 texture{gelb}}
	cylinder {B[1] B[3] r1/3 texture{weiss}}
	cylinder {B[0] B[2] r1/3 texture{gelb}}
 translate Right
}
