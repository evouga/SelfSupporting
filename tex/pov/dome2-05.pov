#include "load/load-common.inc" // declare r1,r2, camera, lights, surroundings

#include "load/dome2-00-vertices.inc" // declare NV,NF, V
#include "load/dome2-00-normals.inc" // declare N

#macro triang(a,b,c) FStriang(a,b,c) #end
#declare i=0; #while (i<NV) #declare V[i]=V[i]-.5*N[i]; #declare i=i+1; #end
SMesh("load/dome2-faces.inc", weiss)
#declare i=0; #while (i<NV) #declare V[i]=V[i]+N[i]; #declare i=i+1; #end
SMesh("load/dome2-faces.inc",  weisst)
		//texture {pigment{color rgbt<1,1,1,.5>} })

#include "load/dome2-05-vertices.inc" 
#include "load/dome2-05-normals.inc" 
SMesh("load/dome2-faces.inc",  concrete)
union { #include "load/dome2-05-weights.inc" rotate -x*90 }

#declare kg=05;
box { #declare ff=0.8;
        <-.82/ff,0,-.82/ff> <.82/ff,-1.64*kg/11*ff*ff,.82/ff>
        translate V[1] rotate -x*90 texture {brownish} }



