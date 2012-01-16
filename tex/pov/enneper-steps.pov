#include "enneper/enneper-common.inc" // declare r1,r2, camera, lights, surroundings


// store Koebe polyhedron vertices in the array VK 
#include "enneper/ek-vertices.inc" // declare NV,NF, V
#declare VK = array[NV];
#declare i=0; #while (i<NV) #declare VK[i]=V[i]; #declare i=i+1; #end

// store 2nd surface in the array VS
#include "enneper/e-vertices.inc" // declare NV,NF, V
#declare VS = array[NV];
#declare i=0; #while (i<NV) #declare VS[i]=V[i]; #declare i=i+1; #end

// compute difference


#declare VM = array[NV];
#declare i=0; #while (i<NV) 
	#declare VM[i]=<VK[i].x, VK[i].y-VS[i].y, VK[i].z>;
#declare i=i+1; #end

/*
#if (clock < 10) 
#declare alph=0;
#end

#if(clock >= 10 & clock < 60)
        #declare alph=clock-10;
#end                          
#if(clock >= 60)
        #declare alph=49;
#end
*/

#declare i=0; #while (i<NV) 
	#declare V[i]=<VM[i].x, VM[i].y-1.3, VM[i].z>;
#declare i=i+1; #end


union {
	#macro Quad(a,b,c,d) EQuad(a,b,c,d) #end
	union { #include "enneper/e-faces.inc" texture {dblau}}
	
	#macro Quad(a,b,c,d) FQuad(a,b,c,d) #end
	mesh { #include "enneper/e-faces.inc" texture {gelb}}

rotate -x*90
} 

#declare alph = 25;

#declare i=0; #while (i<NV) 
	#declare V[i]=<VM[i].x, VK[i].y+((alph/10)*VM[i].y)-3.88, VM[i].z>;
#declare i=i+1; #end

union {
	#macro Quad(a,b,c,d) EQuad(a,b,c,d) #end
	union { #include "enneper/e-faces.inc" texture {weiss}}
	
	#macro Quad(a,b,c,d) FQuad(a,b,c,d) #end
	mesh { #include "enneper/e-faces.inc" texture {dblau}}

rotate -x*90
}   
 
 
#declare i=0; #while (i<NV) 
	#declare V[i]=<VK[i].x, VK[i].y, VK[i].z>;
#declare i=i+1; #end

union {
	#macro Quad(a,b,c,d) EQuad(a,b,c,d) #end
	union { #include "enneper/e-faces.inc" texture {weiss}}
	
	#macro Quad(a,b,c,d) FQuad(a,b,c,d) #end
	mesh { #include "enneper/e-faces.inc" texture { pigment{Gray60}}}

rotate -x*90
}


#declare i=0; #while (i<NV)
        #if(i=206|i=191|i=3|i=207|i=39|i=22)
        union{         
	        cylinder { <VM[i].x, VM[i].y-1.3, VM[i].z> <VK[i].x, VK[i].y, VK[i].z> r2 } 
	        cylinder { <VM[i].x, VM[i].y-1.3, VM[i].z> <VM[i].x, VK[i].y+((alph/10)*VM[i].y)-3.88, VM[i].z> r2 }
                rotate -x*90 texture{pigment{color rgbt<0.2,0.2,0.2,0.7>}  }
	}
	#end
#declare i=i+1; #end                                                                                                                       


#declare i=0; #while (i<NV)
        #if(i=206|i=191|i=3|i=207|i=39|i=22)
                union{ 
                        sphere { <VM[i].x, VM[i].y-1.3, VM[i].z> 0.01}
                        sphere { <VM[i].x, VK[i].y+((alph/10)*VM[i].y)-3.88, VM[i].z> 0.01}        
                        sphere { <VK[i].x, VK[i].y, VK[i].z> 0.01} 
                        texture{pigment{color rgbt<0.2,0.2,0.2,0.7>}  } rotate -x*90
                }
        #end
#declare i=i+1; #end                     
    
    

/*#declare i=0; #while (i<NV) 
        #if(i=206|i=191|i=3|i=207|i=39|i=22)
        text {
                ttf "timrom.ttf" str(i,0,0) .01, 0
                pigment { Red } scale 0.05
                rotate y*90
                rotate z*180
                rotate y*180
                translate V[i]
                rotate -x*90 
                }      
                
                 #end
#declare i=i+1; #end   */