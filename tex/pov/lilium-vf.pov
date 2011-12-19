#include "lilium-ncommon.inc" // declare r1,r2, camera, lights, surroundings


#macro farbe(i)
        //pigment {color rgbt (1-i/4)*<.15,.2,.3,0.2>+(i/4)*<.7,.5,.2,.2>}
        pigment {color rgbt (1-i/4)*<.7,.7,.2,0>+(i/4)*<1,1,1,0>*.7}
        finish {ambient 0.7 diffuse .5}
#end

#include "lilium-n/lnvertices.inc" // declare NV,NF, V

union { #macro triang(a,b,c) Ftriang(a,b,c) #end
	mesh { #include "lilium-n/lnfaces.inc" texture {weiss}}
	rotate -x*90 }

// declare array of faces, by reading file and appropriately
// redefining the triang macro

#declare F = array[NF];
#macro triang(a,b,c) #declare F[i] = <a,b,c>; #declare i=i+1; #end
#declare i=0; #while (i<NF)
	#include "lilium-n/lfaces.inc"  // declare aray, using triang() macro
#end	

// macro for drawing vector field, called in loop

#declare vlen = 0.5;
#macro vf(a,b,c,d,e,f)
	#declare b1 = V[F[i].y]-V[F[i].x];
	#declare b2 = V[F[i].z]-V[F[i].x];
	#declare bary = (V[F[i].x]+ V[F[i].y]+ V[F[i].z])/3;
	#declare trsize = (vlength(V[F[i].x]-bary)+ vlength(V[F[i].y]-bary)+
			vlength(V[F[i].z]-bary))/3;
	cylinder {-<a,b,c>*trsize*vlen,<a,b,c>*trsize*vlen r1
		texture {gelb} translate bary}
	cylinder {-<a,b,c>*trsize*vlen,<a,b,c>*trsize*vlen r1
		texture {gelb} translate bary translate <d,e,f>*r1}
	cylinder {-<a,b,c>*trsize*vlen,<a,b,c>*trsize*vlen r1
		texture {gelb} translate bary translate -<d,e,f>*r1}
	cylinder {-<d,e,f>*trsize*vlen,<d,e,f>*trsize*vlen r1*1.2
		texture {dblau} translate bary}
	#declare i=i+1;
#end

// loop drawing vector fields, NF was defined in include file

union { #declare i=0; #while (i<NF) #include "lilium-n/lilium-n-vf.inc" #end
	rotate -x*90 }
