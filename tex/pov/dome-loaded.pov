#include "load/load-common.inc" // declare r1,r2, camera, lights, surroundings

#include "load/dome-loadedvertices.inc" // declare NV,NF, V
#include "load/dome-normals.inc" // declare N

#macro triang(a,b,c) FStriang(a,b,c) #end
SMesh("load/dome-faces.inc", concrete)

union { union {#include "load/dome-loaded-weights.inc" }
        rotate -x*90 }

box {
        #declare ff=0.8;
        <-.82/ff,0,-.82/ff>
        <.82/ff,-1.64*ff*ff,.82/ff>
        translate V[1]
        rotate -x*90
        texture {brownish}
}

