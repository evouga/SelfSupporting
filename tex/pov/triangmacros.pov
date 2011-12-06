// must declare outside

//#declare M = 6;
//#declare N = 6;
//#declare factor=2.0/(M+N);


// define array "pkte"

#macro declarepoints(Dev)
#declare pkte=array[M][N];
#declare i=0; #while (i<M)
        #declare j=0; #while (j<N)
        #declare pkte[i][j] = factor*(<2,0,0>*(i-M/2)
                + <-sqrt(3)/2,sqrt(3),0>*(j-N/2)
                + <Rand_Gauss(0,Dev,1),Rand_Gauss(0,Dev,1),0>);
        #declare j=j+1; #end
#declare i=i+1; #end
#end

// showpoints


#macro showpoints(P,anz1,anz2,rad)
 #declare i=0; #while (i<anz1)
  #declare j=0; #while (j<anz2) 
    #if (!(i=0 & j = anz2-1) & !(i=anz1-1 & j=0))
	sphere {P[i][j], rad} #end
  #declare j=j+1; #end
 #declare i=i+1; #end
#end


// show edges

#macro showedges(P,anz1,anz2,rad)
#declare i=0; #while (i<anz1)
	#declare j=0; #while (j<anz2)
	#if (j<anz2-1 & i>0 & i<anz1-1)
		cylinder { P[i][j], P[i][j+1] rad } #end
	#if (i<anz1-1 & j>0 & j< anz2-1) 
		cylinder { P[i][j], P[i+1][j] rad }  
	#end
	#if (i<anz1-1 & j<anz2-1 & !(i=0 &  j=anz2-2) & !(i=anz1-2 & j=0))
		cylinder { P[i][j], P[i+1][j+1] rad} 
	#end
	#declare j=j+1; #end
  #declare i=i+1; #end
#end

#macro showfaces(P,anz1,anz2)
#declare i=0; #while (i<anz1-1)
	#declare j=0; #while (j<anz2-1)
    	#if (!(i=anz1-2 & j=0))
		triangle { P[i][j], P[i+1][j], P[i+1][j+1]}
	#end
    	#if (!(i=0& j=anz2-2))
		triangle { P[i][j], P[i+1][j+1], P[i][j+1]}
	#end
	#declare j=j+1; #end
  #declare i=i+1; #end
#end

// compute circumcenters

#macro declarecircums()

#declare circums1 = array[M-1][N-1]
#declare circums2 = array[M-1][N-1]

#declare i=0; #while (i<M-1)
	#declare j=0; #while (j<N-1)
		#declare U = pkte[i+1][j]-pkte[i][j];
		#declare UR = vrotate(U,<0,0,90>);
		#declare V = pkte[i+1][j+1]-pkte[i][j];
		#declare Lam = vdot(V-U,V)/ vdot(UR,V);
		#declare circums1[i][j] = pkte[i][j] + (Lam*UR+U)/2;

		#declare U = pkte[i][j+1]-pkte[i][j];
		#declare UR = vrotate(U,<0,0,90>);
		#declare Lam = vdot(V-U,V)/ vdot(UR,V);
		#declare circums2[i][j] = pkte[i][j] + (Lam*UR+U)/2;

		//sphere {pkte[i][j]+V,2*r1}
		//#declare rr = vlength(m-pkte[i][j]);
		//sphere {m,2*r1}
		//cylinder{m,m+.3*z,rr}	
		//object {torus {rr 0.1} rotate x*90 translate m}
	#declare j=j+1; #end
#declare i=i+1; #end
#end

// show dual hexagons

#macro showdualedges(Q1,Q2,anz1,anz2,rad)
#declare i=0; #while (i<M-2)
	#declare j=0; #while (j<N-2)
		#declare PP=array[7]  {
			Q1[i][j], Q2[i][j], Q1[i][j+1], Q2[i+1][j+1],
			Q1[i+1][j+1], Q2[i+1][j], Q1[i][j]}
		#declare ii=0; #while (ii<6) cylinder{PP[ii], PP[ii+1], rad}
		#declare ii=ii+1;  #end
	#declare j=j+1; #end
#declare i=i+1; #end
#end

#macro showdualfaces(Q1,Q2,anz1,anz2)
#declare i=0; #while (i<M-2)
	#declare j=0; #while (j<N-2)
		#declare PP=array[6]  {
			Q1[i][j], Q2[i][j], Q1[i][j+1], Q2[i+1][j+1],
			Q1[i+1][j+1], Q2[i+1][j]}
		triangle{PP[0],PP[1],PP[2]}
		triangle{PP[0],PP[2],PP[3]}
		triangle{PP[0],PP[3],PP[4]}
		triangle{PP[0],PP[4],PP[5]}
	#declare j=j+1; #end
#declare i=i+1; #end
#end

// points on the Maxwell paraboloid

#macro movepointstomaxwell(P)
#declare i=0; #while (i<M)
	#declare j=0; #while (j<N)
		#declare P[i][j]=P[i][j]+
			vdot(P[i][j],P[i][j])*z/2;
	#declare j=j+1; #end
#declare i=i+1; #end
#end


// points in Maxwell paraboloid's tangent planes

#macro movedualpointstomaxwell(C1,C2)
#declare i=0; #while (i<M-1)
	#declare j=0; #while (j<N-1)
		#declare C1[i][j] = C1[i][j] +(pkte[i][j].z 
			+pkte[i][j].x * (C1[i][j].x-pkte[i][j].x)
			+pkte[i][j].y * (C1[i][j].y-pkte[i][j].y))*z;
		#declare C2[i][j] = C2[i][j] +(pkte[i][j].z 
			+pkte[i][j].x * (C2[i][j].x-pkte[i][j].x)
			+pkte[i][j].y * (C2[i][j].y-pkte[i][j].y))*z;
	#declare j=j+1; #end
#declare i=i+1; #end
#end


