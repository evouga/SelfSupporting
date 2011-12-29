#include "lilium-ncommon.inc" // declare r1,r2, camera, lights, surroundings


// store Koebe polyhedron vertices in the array VK 
#include "enneper/ek-vertices.inc" // declare NV,NF, V
#declare VK = array[NV];
#declare i=0; #while (i<NV) #declare VK[i]=V[i]; #declare i=i+1; #end

// store 2nd surface in the array VS
#include "enneper/e-vertices.inc" // declare NV,NF, V
#declare VS = array[NV];
#declare i=0; #while (i<NV) #declare VS[i]=V[i]; #declare i=i+1; #end


// compute intermediate surface


#declare tt = -2.0;
#while (tt<2.01)
	#declare i=0; #while (i<NV) 
		#declare V[i]=((1-tt)*VK[i]+tt*VS[i])/3
			+<-.15,0,.8>*(tt)*.5 +y*.5;
	#declare i=i+1; #end

	union {
		#macro Quad(a,b,c,d) EQuad(a,b,c,d) #end
		union { #include "enneper/e-faces.inc" texture {weiss}}
	
		#macro Quad(a,b,c,d) FQuad(a,b,c,d) #end
		mesh { #include "enneper/e-faces.inc" texture {gelb}}

	rotate -x*90
	}
#declare tt=tt+2.0;
#end




