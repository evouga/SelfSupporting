#include "colors.inc"
#include "farben.inc"
#include "mycolors.inc"
#include "rand.inc"

camera {
	//orthographic
	location <-1,0,0.1>*35 rotate z*(47)
		translate <0.5,1,0>
	angle 35
	look_at <0,0,2>
	sky <0,0,1>
}


light_source { <-10,-20,30>*25 color rgb <1,1,1> *.6 
	//shadowless
	area_light <20,0,0>*40 <0,20,0>*40 5 5 jitter circular 
	}
light_source { <-10,-10,20>*25 color rgb <1,1,1> *0.7 shadowless}
light_source { <0,0,-20>*25 color rgb <1,1,1> *0.8 shadowless}


//plane {z,0 texture {weiss}}
background {color rgb <1,1,1>}
box {<-50,-50,-2> <10,10,0.3> texture {weiss}}
#declare r1=0.002;
#declare r2=0.002;

#include "facemacros.inc"

#declare MySpline = spline { cubic_spline 
	0 <0,0,1> 1 <0,0,1> 2 <0,1,1> 3 <0,1,0> 4 <1,1,0> 5 <1,0,0> 6 <1,0,0>}


#macro edgecolor(T) MySpline(T) #end
 #macro WEdge(i,j,w)
	#declare fff=pow(w*r1*vlength(V[i]-V[j]),.5);
	cylinder{V[i] V[j] fff*.01+0.007
	texture {pigment {color rgb edgecolor(fff*.9)}}}
 #end
 #macro w2edge(i,j,w)
	#declare fff=pow(w*r1*vlength(V[i]-V[j]),.5);
	cylinder{V[i] V[j] fff*.01+0.007
	texture {pigment {color rgb edgecolor(fff*.9)}}
	}
 #end

