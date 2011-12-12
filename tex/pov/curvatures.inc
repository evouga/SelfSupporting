// the only difference in curvature.pov and curvature1.pov is
// the viewpoint. The common body of the scene is stored
// in this file (curvatures.inc)


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
//#macro SS() (1.0+cos(2*x)/3-(y*y)/3-(x*y)/2) #end
#macro SS() (1.0-x*y/2+cos(2*x-3*y)/20) #end
#declare ff = function(x,y) { FF() };
#declare ss = function(x,y) { SS() };
#declare dx=0.1;
#declare dy=0.1;
#declare sx = function(x,y) {(ss(x+dx,y)-ss(x-dx,y))/(2*dx)};
#declare sy = function(x,y) {(ss(x,y+dy)-ss(x,y-dy))/(2*dy)};
#declare fx = function(x,y) {x};
#declare fy = function(x,y) {y};
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
#macro bcyl() cylinder{-2*z,2*z,1} #end

/*
	isosurface { function {FF() - z}
		open max_gradient 10
		contained_by {bb(1)}
		texture {weiss}
		clipped_by {bcyl()}
	}
	isosurface { function {SS() - z}
		open max_gradient 10
		contained_by {bb(1)}
		texture {gelb}
		clipped_by {bcyl()}
	}
*/

#declare du=.2;
#declare dv=du;

#macro makemesh(F,Fx,Fy)
#declare uu = -1.1; #while (uu<1.1)
	#declare vv = -1.1; #while (vv<1.1)
		#declare P0=<uu,vv,F(uu,vv)>;
		#declare N0=<Fx(uu,vv),Fy(uu,vv),-1>;
		#declare P1=<uu+du,vv,F(uu+du,vv)>;
		#declare N1=<Fx(uu+du,vv),Fy(uu+du,vv),-1>;
		#declare P2=<uu+du,vv+dv,F(uu+du,vv+dv)>;
		#declare N2=<Fx(uu+du,vv+dv),Fy(uu+du,vv+dv),-1>;
		#declare P3=<uu,vv+dv,F(uu,vv+dv)>;
		#declare N3=<Fx(uu,vv+dv),Fy(uu,vv+dv),-1>;
		smooth_triangle {P0,N0,P1,N1,P2,N2}
		smooth_triangle {P0,N0,P2,N2,P3,N3}
	#declare vv=vv+dv; #end
#declare uu=uu+du; #end
#end

mesh{ makemesh(ss,sx,sy) texture{gelb} clipped_by {bcyl()}}
mesh{ makemesh(ff,fx,fy) texture{weiss} clipped_by {bcyl()}}

#declare du=.2;
#declare dv=du;
#declare len=.1;
#declare rad=.005;

#declare uu = -1; #while (uu<1.1)
	#declare vv = -1; #while (vv<1.1)
		#if (uu*uu+vv*vv<1)
		#declare H=<sxx(uu,vv), sxy(uu,vv), syy(uu,vv)>;
		#declare D=<sx(uu,vv),sy(uu,vv)>;
		#declare zz=ss(uu,vv);
		#declare V1=ev1(H,D)*len;
		#declare V2=ev2(H,D)*len;
		cylinder {-V1,V1 rad translate <uu,vv,zz>
			texture {brownish} }
		cylinder {-V2,V2 rad translate <uu,vv,zz>
			texture {brownish} }

		#declare D=<uu,vv>;
		#declare zz=ff(uu,vv);
		#declare V1=ev1(H,D)*len;
		#declare V2=ev2(H,D)*len;
		cylinder {-V1,V1 rad/2 translate <uu,vv,zz>
			texture {dblau} translate z*rad*3}
		cylinder {-V2,V2 rad/2 translate <uu,vv,zz>
			texture {dblau} translate z*rad*3}
		#end
	#declare vv=vv+dv; #end
#declare uu=uu+du; #end
