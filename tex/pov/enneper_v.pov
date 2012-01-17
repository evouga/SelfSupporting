#include "enneper/enneper-acommon.inc" // declare r1,r2, camera, lights, surroundings

#declare fact=1.5;

// store Koebe polyhedron vertices in the array VK 
#include "enneper/ek-vertices.inc" // declare NV,NF, V
#declare VK = array[NV];
#declare i=0; #while (i<NV) #declare VK[i]=V[i]*fact; #declare i=i+1; #end

// store 2nd surface in the array VS
#include "enneper/e-vertices.inc" // declare NV,NF, V
#declare VS = array[NV];
#declare i=0; #while (i<NV) #declare VS[i]=V[i]*fact; #declare i=i+1; #end


// declare factors for linear combination plus vertical shift
// for each mesh whichi s combined from VK and VS

#declare NMeshes = 3;
#declare comb = array[NMeshes] {
	<1.5,-1.5>
	<1,0>,
	<3,-2>,
}
#declare Origin = array[NMeshes] {
	<0,-.7,-.7>,
	<0,-.7,0>,
	<0,-.7,.6>
}

// store textures

#macro FText(i) #if (i<1) weiss #else hellgelb #end #end
#macro EText(i) #if (i<1) dblau #else schwarz #end #end

// render intermediate surfaces

#declare j=0;
#while (j<NMeshes)
	#declare i=0; #while (i<NV) 
		#declare V[i]= < VK[i].x,
			comb[j].x*VK[i].y +comb[j].y*VS[i].y,
			VK[i].z>/3;
	#declare i=i+1; #end

	union {
		#macro Quad(a,b,c,d) EQuad(a,b,c,d) #end
		union { #include "enneper/e-faces.inc" texture {EText(j)}}
	
		#macro Quad(a,b,c,d) FQuad(a,b,c,d) #end
		mesh { #include "enneper/e-faces.inc" texture {FText(j)}}

		rotate -x*90
		rotate z*20
		translate Origin[j]
	}
	#declare j=j+1;
#end

////// LOAD NEXT MESH

#declare fact=0.6;

// store Koebe polyhedron vertices in the array VK 
#include "enneper/enneper_asymm_vertices.inc" // declare NV,NF, V
#declare VK = array[NV];
#declare i=0; #while (i<NV) #declare VK[i]=V[i]*fact; #declare i=i+1; #end

// store Christoffel dual surface in the array VM
#include "enneper/enneper_asymm_christoffelvertices.inc" // declare NV,NF, V
#declare VM = array[NV];
#declare i=0; #while (i<NV) #declare VM[i]=V[i]*fact; #declare i=i+1; #end

#declare NMeshes = 1;
#declare comb = array[NMeshes] {
	<-1,8>,
}
#declare Origin = array[NMeshes] {
	<0,1,0.5>
}

// store textures

#macro FText(i) hellgelb #end
#macro EText(i) schwarz #end

// render intermediate surfaces

#declare j=0;
#while (j<NMeshes)
	object {
	#declare i=0; #while (i<NV) 
		#declare V[i]= < VK[i].x, VK[i].y,
		VK[i].z*comb[j].x + VM[i].z*comb[j].y>;
	#declare i=i+1; #end

	union {
		#macro Quad(a,b,c,d) EQuad(a,b,c,d) #end
		union { #include "enneper/enneper_asymm_faces.inc"
			texture {EText(j)}}
	
		#macro Quad(a,b,c,d) FQuad(a,b,c,d) #end
		union { #include "enneper/enneper_asymm_faces.inc"
			texture {FText(j)}}

	}
	rotate -z*40
	translate Origin[j]
	}
	#declare j=j+1;

#end

