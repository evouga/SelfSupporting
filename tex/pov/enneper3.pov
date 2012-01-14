#include "enneper/enneper-common.inc" // declare r1,r2, camera, lights, surroundings


// store Koebe polyhedron vertices in the array VK 
#include "enneper/ek-vertices.inc" // declare NV,NF, V
#declare VK = array[NV];
#declare i=0; #while (i<NV) #declare VK[i]=V[i]; #declare i=i+1; #end

// store 2nd surface in the array VS
#include "enneper/e-vertices.inc" // declare NV,NF, V
#declare VS = array[NV];
#declare i=0; #while (i<NV) #declare VS[i]=V[i]; #declare i=i+1; #end


// declare factors for linear combination plus vertical shift
// for each mesh whichi s combined from VK and VS

#declare NMeshes = 4;
#declare comb = array[NMeshes] {
	<5,-4,0>,
	<3,-2,0>,
	<1,0,0>,
	<1,-1,+.5>
}

// store textures

#macro FText(i) #if (i<3) gelb #else weiss #end #end
#macro EText(i) #if (i<3) weiss #else dblau #end #end

// render intermediate surfaces

#declare j=0;
#while (j<NMeshes)
	#declare i=0; #while (i<NV) 
		#declare V[i]= < VK[i].x,
			comb[j].x*VK[i].y +comb[j].y*VS[i].y+comb[j].z,
			VK[i].z>/3
		+ <-.15,0,.8>*(j-1.6) +y*0.5;
	#declare i=i+1; #end

	union {
		#macro Quad(a,b,c,d) EQuad(a,b,c,d) #end
		union { #include "enneper/e-faces.inc" texture {EText(j)}}
	
		#macro Quad(a,b,c,d) FQuad(a,b,c,d) #end
		mesh { #include "enneper/e-faces.inc" texture {FText(j)}}

	rotate -x*90
	}
	#declare j=j+1;
#end

