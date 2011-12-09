#include "colors.inc"
#include "farben.inc"
#include "mycolors.inc"
#include "rand.inc"

camera {
	orthographic
	location <-1,0,0.5>*4 rotate z*20
	//location <0,0,6>
	angle 45
	look_at <0,0,0.5>
	sky <0.01,0,1>
}

#include "curvatures.inc"

/*

light_source { <-10,-20,30> color rgb <1,1,1> *.6 
	//shadowless
	area_light <20,0,0> <0,20,0> 5 5 jitter circular 
	}
light_source { <0,0,20> color rgb <1,1,1> *0.7 shadowless}
light_source { <0,0,-20> color rgb <1,1,1> *0.7 shadowless}


plane {z,-0.01 texture{weiss}}
//box {<-1.1,-1.1,-0.3><1.1,1.1,-0.2> rotate z*10 texture {gelb}}
background{White}

#macro FF() ((x*x+y*y)/2) #end
#macro SS() (1.0+cos(2*x)/3-(y*y)/3-(x*y)/2) #end
#declare ff = function(x,y) { FF() } ;
#declare ss = function(x,y) { SS() } ;
#declare dx=0.1;
#declare dy=0.1;
#declare sx = function(x,y) {(ss(x+dx,y)-ss(x-dx,y))/(2*dx)};
#declare sy = function(x,y) {(ss(x,y+dy)-ss(x,y-dy))/(2*dy)};
#declare sxx = function(x,y) {(ss(x+dx,y)-2*ss(x,y)+ss(x-dx,y))/(dx*dx)};
#declare sxy = function(x,y) {(sy(x+dx,y)-sy(x-dx,y))/(2*dy)}
#declare syy = function(x,y) {(ss(x,y+dy)-2*ss(x,y)+ss(x,y-dy))/(dy*dy)};
#declare det = function (a,b,c,d) {a*d-b*c}
#declare tr = function (a,d) {(a+d)/2}
#declare eval1=function(a,b,c,d) {tr(a,d)+pow(pow(tr(a,d),2)-det(a,b,c,d),.5)}
#declare eval2=function(a,b,c,d) {tr(a,d)-pow(pow(tr(a,d),2)-det(a,b,c,d),.5)}
#macro ev1(H,D)  vnormalize(
	H.y*<1,0,D.x> + (eval1(H.x,H.y,H.y,H.z)-H.x)*<0,1,D.y>) #end
#macro ev2(H,D)  vnormalize(
	H.y*<1,0,D.x> + (eval2(H.x,H.y,H.y,H.z)-H.x)*<0,1,D.y>) #end
//#macro ev3(H,D)  vnormalize(
//	(H.x-eval1(H.x,H.y,H.y,H.z))*<1,0,D.x> + H.y*<0,1,D.y>) #end

#macro bb(f) box{<-1,-1,0>*f,<1,1,2>*f} #end
#macro bcyl(r) cylinder{-2*z,2*z,1} texture {pigment{color rgbt<1,1,1,1>}} #end

intersection {
	union{
	isosurface { function {FF() - z}
		open max_gradient 10
		contained_by {bb(1)}
		texture {weiss}
	}
	isosurface { function {SS() - z}
		open max_gradient 10
		contained_by {bb(1)}
		texture {gelb}
	}

#declare du=.2;
#declare dv=du;
#declare len=.1;
#declare rad=.005;

#declare uu = -1; #while (uu<1.1)
	#declare vv = -1; #while (vv<1.1)
		#declare H=<sxx(uu,vv), sxy(uu,vv), syy(uu,vv)>;
		#declare D=<sx(uu,vv),sy(uu,vv)>;
		#declare zz=ss(uu,vv);
		#declare V1=ev1(H,D)*len;
		#declare V2=ev2(H,D)*len;
		cylinder {-V1,V1 rad translate <uu,vv,zz>
			texture {brownish} }
		cylinder {-V2,V2 rad translate <uu,vv,zz>
			texture {brownish} }
	#declare vv=vv+dv; #end
#declare uu=uu+du; #end

} // end union
	bcyl(1)
} // end intersection
*/
