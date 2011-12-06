#include "colors.inc"

#declare grau_schwein = texture  { pigment { color rgb < 1,1,0.95> }
                            finish { ambient 0.2 diffuse 0.8 phong 1 }};

#declare dblau_glanz = texture {  pigment { color rgb <0.1,0.1,0.4> }
                             finish { ambient 0.2 diffuse 0.8 phong 1 }};

#declare schwarz = texture { pigment { Black }};
// unused

/*
#declare lila = rgb <0.75, 0.5, 1.0>;


#declare folie_schwarz = texture { pigment { color rgbf < 0, 0, 0, 0.6>}};
#declare folie_trans = texture { pigment { color rgbf < 0.9, 0.9, 0.9, 0.6>}}; 
#declare folie_blau = texture { pigment { color rgbf < 0.7, 0.7, 1, 0.8>}};
#declare folie_rot = texture { pigment { color rgbf < 1, 0.7, 0.7, 0.2>}};
#declare folie_gelb = texture { pigment { color rgbf < 1, 1, 0.5, 0.8>}};
#declare folie_gruen = texture { pigment { color rgbf < 0.7, 1, 0.7, 0.8>}};

#declare schwarz_glanz = texture  { pigment { Black }
                            finish { ambient 0.2 diffuse 0.8 phong 1 }};
#declare rot_glanz = texture  { pigment { Red }
                            finish { ambient 0.2 diffuse 0.8 phong 1 }};
#declare gruen_glanz = texture  { pigment { Green }
                            finish { ambient 0.2 diffuse 0.8 phong 1 }};
#declare blau_glanz = texture {  pigment { Blue }
                             finish { ambient 0.2 diffuse 0.8 phong 1 }};
#declare gelb_glanz = texture {  pigment { Yellow }
                             finish { ambient 0.2 diffuse 0.8 phong 1 }};    
*/
#declare gagerlgelb_glanz = texture {  pigment { rgb <1,0.9,0.7> }
                             finish { ambient 0.2 diffuse 0.8 phong 1 }};                             
/*
#declare blech_glanz = texture {  pigment { LightBlue }
                             finish { ambient 0.2 diffuse 0.8 phong .5 }};                             
             
#declare orangerot_glanz = texture {  pigment { OrangeRed }
                             finish { ambient 0.2 diffuse 0.8 phong .5 }};    
#declare orange_glanz = texture {  pigment { Orange }
                             finish { ambient 0.2 diffuse 0.8 phong .5 }};    
#declare mandarinorange_glanz = texture {  pigment { MandarinOrange  }        
                             finish { ambient 0.2 diffuse 0.8 phong .5 }};    
#declare neonblau_glanz = texture {  pigment { NeonBlue }
                             finish { ambient 0.2 diffuse 0.8 phong 1 }};                             
#declare stahlblau_glanz = texture {  pigment { LightSteelBlue }
                             finish { ambient 0.2 diffuse 0.8 phong 1 }};  
#declare himmelblau_glanz = texture {  pigment { SkyBlue }
                             finish { ambient 0.2 diffuse 0.8 phong 1 }};                                                         
                             
                                                        
                                                     
             
#declare rot = texture { pigment { Red }};
#declare blau = texture { pigment { Blue }};
#declare gruen = texture { pigment { Green }};
#declare gelb = texture { pigment { Yellow }};
#declare blech = texture { pigment { color LightBlue }
                             normal { bumps 0.4 scale 0.2 }
                             finish { phong 1}};  
#declare neonblau = texture { pigment { NeonBlue }};     
#declare hellblau = texture { pigment { LightBlue }};                     

#declare holz1 = texture { pigment { wood color_map 
	{ [0.0 color DarkTan] [0.9 color DarkBrown] [1.0 color VeryDarkBrown]}
       turbulence 0.05 scale <0.2, 0.3, 1>} finish { phong 1 }};
//#declare holz2 = texture { pigment { DMFWood4 scale 4 } finish { Shiny } };

#declare marmor1 = texture { pigment {
                                    marble
                                    turbulence 5
                                    color_map {
                                     [0.0  color Pink]
                                     [1.0  color LightGray]
                                     [0.5 color LightBlue]
                                     }}};

*/


#declare gelb = texture { pigment {rgb <0.7,0.5,0.2>}
	finish {ambient 0.7}}
#declare rot = texture { pigment {rgb <0.7,0.2,0.25>}
	finish {ambient 0.7}}
// #declare hgelb = texture { pigment {rgbt <0.91,0.85,0.76,0.2>}
#declare hgelb = texture { pigment {rgb <0.7,0.5,0.2>*0.5+<1,1,1>*0.5}
	finish {ambient 0.9}}
#declare weiss = texture { pigment {rgb <1,1,1>}
	finish {ambient 0.4}}
#declare weisst = texture { pigment {rgbt <1,1,1,0.5>}
	finish {ambient 0.4}}
#declare weiss2 = texture { pigment {rgbt <0.8,0.8,0.8,0.2>}
	finish {ambient 0.7}}
#declare nicht = texture { pigment {rgbt <1,1,1,1>}
	finish {ambient 0.7}}
#declare blau = texture { pigment {rgb <0.1, 0.2, 0.4>}
	finish {ambient 0.7}}
#declare dblau = texture { pigment {rgb <0.15, 0.2, 0.3>}
	finish {ambient 0.7}}


#declare tt = 0.3;
#declare xgelb = texture {pigment {rgb<.7,.5,.2>*(1-tt)+<1,1,1>*tt}
	finish {ambient .2 diffuse .6 phong .7 reflection 0.01}}
#declare xweiss = texture {pigment {rgbt<1,1,1,.5>}
	finish {ambient .2 diffuse .9 phong .7 reflection 0.01}}
#declare tgelb = texture {pigment {rgbt<.7,.5,.2,.6>*(1-tt)+<1,1,1,.6>*tt}
	finish {ambient .2 diffuse .6 phong .7 reflection 0.01}}

#declare hblau=texture{pigment{color rgb<.2,.4,.8>}
        finish {ambient .5}}
#declare orange=texture{pigment{color rgb<.8,.4,.2>}
        finish {ambient .5}}

