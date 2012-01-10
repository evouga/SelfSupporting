#include "colors.inc"
#include "farben.inc"
#include "mycolors.inc"
#include "rand.inc"

camera {
	orthographic
	location <-1,0,.3>*110 rotate z*(00)
	angle 40
	look_at <0,0,0>
	sky <0,0,1>
}


#declare ff=15;
light_source { <-10,-20,30>*ff color rgb <1,1,1> *.6 
	//shadowless
	area_light <20,0,0>*ff <0,20,0>*ff 5 5 jitter circular 
	}
light_source { <-10,-10,20>*ff color rgb <1,1,1> *0.7 shadowless}
light_source { <0,0,-20>*ff color rgb <1,1,1> *0.8 shadowless}


//plane {z,-0.0 texture {weiss}}
background {color rgb <1,1,1>}
box {<-1,-1,-1>*10*ff <1.5,1,0>*2.5*ff texture {weiss}}
#declare r1=0.002*ff;
#declare r2=0.002*ff;

#macro Etriang(a,b,c) cylinder {V[a] V[b] r2} cylinder {V[b] V[c] r2}
		cylinder {V[c] V[a] r2} #end

#macro Ftriang(a,b,c) triangle {V[a] V[b] V[c]} #end


#macro EQuad(a,b,c,d) 
		cylinder {V[a] V[b] r2} cylinder {V[b] V[c] r2}
		cylinder {V[c] V[d] r2} cylinder {V[d] V[a] r2} #end
#macro EHex(a,b,c,d,e,f) 
		cylinder {V[a] V[b] r2} cylinder {V[b] V[c] r2}
		cylinder {V[c] V[d] r2} cylinder {V[d] V[e] r2}
		cylinder {V[e] V[f] r2} cylinder {V[f] V[a] r2} #end

#macro FQuad(a,b,c,d) triangle {V[a] V[b] V[c]}
		triangle {V[a] V[c] V[d]} #end
#macro FHex(a,b,c,d,e,f)
		triangle {V[a] V[b] V[c]} triangle {V[a] V[c] V[d]}
		triangle {V[a] V[d] V[e]} triangle {V[a] V[e] V[f]} #end
