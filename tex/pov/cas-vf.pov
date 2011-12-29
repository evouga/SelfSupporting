#include "cas/cas-common.inc" // declare r1,r2, camera, lights, surroundings



#include "cas/cas-nvertices.inc" // declare NV,NF, V

union { 
	#macro Tri(a,b,c) Ftriang(a,b,c) #end
	#macro Quad(a,b,c,d) FQuad(a,b,c,d) #end
	mesh { #include "cas/casfaces.inc" texture {weiss}}
	rotate -x*90 }

// declare array of faces, by reading file and appropriately
// redefining the Tri and Quad macros

#declare F = array[NF];
#macro Tri(a,b,c) #declare F[i] = <a,b,c>; #declare i=i+1; #end
#macro Quad(a,b,c,d) #declare F[i] = <a,b,c>; #declare i=i+1; #end
#declare i=0; #while (i<NF)
	#include "cas/casfaces.inc"  // declare array
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
		texture {brownish} translate bary}
	//cylinder {-<a,b,c>*trsize*vlen,<a,b,c>*trsize*vlen r1
	//	texture {gelb} translate bary translate <d,e,f>*r1}
	//cylinder {-<a,b,c>*trsize*vlen,<a,b,c>*trsize*vlen r1
	//	texture {gelb} translate bary translate -<d,e,f>*r1}
	cylinder {-<d,e,f>*trsize*vlen,<d,e,f>*trsize*vlen r1*1.2
		texture {dblau} translate bary}
	#declare i=i+1;
#end

// loop drawing vector fields, NF was defined in include file

union { #declare i=0; #while (i<NF) #include "cas/cas-n-vf.inc" #end
	rotate -x*90 }
