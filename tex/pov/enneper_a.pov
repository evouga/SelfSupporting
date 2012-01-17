#include "enneper/enneper-acommon.inc" // declare r1,r2, camera, lights, surroundings


// store Koebe polyhedron vertices in the array VK 
#include "enneper/enneper_asymm_vertices.inc" // declare NV,NF, V
#declare VK = array[NV];
#declare i=0; #while (i<NV) #declare VK[i]=V[i]; #declare i=i+1; #end

// store Christoffel dual surface in the array VM
#include "enneper/enneper_asymm_christoffelvertices.inc" // declare NV,NF, V
#declare VM = array[NV];
#declare i=0; #while (i<NV) #declare VM[i]=V[i]; #declare i=i+1; #end

#declare hellgelb = texture {  pigment { rgb <1,0.9,0.7> }
                             finish { ambient 0.4 diffuse 0.6 phong 1 }};  
#declare NMeshes = 4;
#declare comb = array[NMeshes] {
	<0,5>,
	<-1,0>,
	<-1,8>,
	<-1,-8>,
}
#declare Origin = array[NMeshes] {
	<0,-1,0>,
	<0,-3,1>
	<0,1,1>
	<0,3,1>
}

// store textures

#macro FText(i) #if (i<1) weiss #else hellgelb #end #end
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
	rotate -z*30
	translate Origin[j]
	}
	#declare j=j+1;

#end

