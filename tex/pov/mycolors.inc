#declare mesh_white = texture { pigment {rgb <1,1,1>}
       finish {ambient 0.4}}
#declare mesh_transp = texture {pigment {color rgbt<1,1,1,.1>}}
#declare brownish = texture {pigment {rgb<.7,.5,.2>*.7+<1,1,1>*.3}
           finish {ambient .2 diffuse .6 phong .7 reflection 0.01}}
#declare darkblue = texture { pigment {rgb <0.15, 0.2, 0.3>}
       finish {ambient 0.7}}
#declare concrete = texture {pigment {color rgb <1,1,.9> }  
        finish {ambient 0.2 diffuse 0.6}}


