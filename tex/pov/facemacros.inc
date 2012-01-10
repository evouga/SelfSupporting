
#macro Etriang(a,b,c) cylinder {V[a] V[b] r2} cylinder {V[b] V[c] r2}
		cylinder {V[c] V[a] r2} #end

#macro Ftriang(a,b,c) triangle {V[a] V[b] V[c]} #end


#macro EQuad(a,b,c,d) 
		cylinder {V[a] V[b] r2} cylinder {V[b] V[c] r2}
		cylinder {V[c] V[d] r2} cylinder {V[d] V[a] r2} #end
#macro EPent(a,b,c,d,e) 
		cylinder {V[a] V[b] r2} cylinder {V[b] V[c] r2}
		cylinder {V[c] V[d] r2} cylinder {V[d] V[e] r2}
		cylinder {V[e] V[a] r2} #end
#macro EHex(a,b,c,d,e,f) 
		cylinder {V[a] V[b] r2} cylinder {V[b] V[c] r2}
		cylinder {V[c] V[d] r2} cylinder {V[d] V[e] r2}
		cylinder {V[e] V[f] r2} cylinder {V[f] V[a] r2} #end

#macro FQuad(a,b,c,d) triangle {V[a] V[b] V[c]}
		triangle {V[a] V[c] V[d]} #end
#macro FPent(a,b,c,d,e)
		triangle {V[a] V[b] V[c]} triangle {V[a] V[c] V[d]}
		triangle {V[a] V[d] V[e]} #end
#macro FHex(a,b,c,d,e,f)
		triangle {V[a] V[b] V[c]} triangle {V[a] V[c] V[d]}
		triangle {V[a] V[d] V[e]} triangle {V[a] V[e] V[f]} #end
