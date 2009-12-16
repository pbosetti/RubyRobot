#require 'gsl'
require 'matrix'
require 'yaml'

#include GSL
#include GSL::MultiMin
#include Math

module Optimizer

class PPOcubicspline

	def initialize(r,filename,vi,ai,vf,af,tq,dT)
		crosspoints = YAML::load_file(filename)
		crossjoints = Array.new(0,Hash.new)
		crosspoints.each do |cp|
			cj = cp.delete(:time)
		  	puts "Out of range!" if !r.ik(cp)
			crossjoints << r.joints.push(cj)
		end
		np = crossjoints.length
		puts np
		m    = Matrix.rows(crossjoints)
	
	optimized = false
	puts(" Step | k | Torque 1 | Torque 2 | Torque 3 | Torque 4\n")
	puts("-------------------------------------------------------------\n")
	counter = 0	
	while !optimized
		h    = Array.new(np+1)
		tm   = Array.new(np+2)
		capp = Array.new(np)
		q    = Array.new()
		acc  = Array.new()
		ajoints = Array.new()
		vjoints = Array.new()
		joints  = Array.new()
		a = Array.new()	
		c = Array.new()		
		counter += 1
		h[0] = (m[1,4]-m[0,4])/2.0
		h[1] = h[0]
		for i in 1..np-3
			h[i+1] = m[i+1,4]-m[i,4]
		end
		h[np-1] = (m[np-1,4]-m[np-2,4])/2.0
		h[np] 	= h[np-1]
		tm[0]   = m[0,4]
		for i in 0..np
			tm[i+1] = tm[i]+h[i]
		end
		ap = Array.new(np,0)
		ap[0] = 3.0*h[0]+2.0*h[1]+h[0]**2/h[1]
		ap[1] = h[1]
		a << ap
		ap = Array.new(np,0)
		ap[0] = h[1]-h[0]**2/h[1]
		ap[1] = 2.0*(h[1]+h[2])
		ap[2] = h[2]
		a << ap
		for i in 2..np-3
			ap = Array.new(np,0)
			ap[i-1] = h[i]
			ap[i] 	 = 2.0*(h[i]+h[i+1])
			ap[i+1] = h[i+1]
			a << ap
		end
		ap = Array.new(np,0)
		ap[np-3] = h[np-2]
		ap[np-2] = 2.0*(h[np-2]+h[np-1])
		ap[np-1] = h[np-1]-h[np]**2/h[np-1]
		a << ap
		ap = Array.new(np,0)
		ap[np-2] = h[np-1]
		ap[np-1] = 2.0*h[np-1]+h[np]**2/h[np-1]+3.0*h[np]
		a << ap
		a = Matrix.rows(a)
		
		for i in 0..3
			capp = Array.new()
			capp[0] = 6.0*(m[1,i]/h[1]+m[0,i]/h[0])-6.0*(1/h[0]+1/h[1])*(m[0,i]+h[0]*vi[i]+h[0]**2/3.0*ai[i])-h[0]*ai[i]
			capp[1] = 6.0/h[1]*(m[0,i]+h[0]*vi[i]+h[0]**2/3.0*ai[i])+6.0*m[2,i]/h[2]-6.0*(1/h[1]+1/h[2])*m[1,i]
			for n in 2..np-3
				capp[n] = 6.0*((m[n+1,i]-m[n,i])/h[n+1]-(m[n,i]-m[n-1,i])/h[n])
			end
			capp[np-2] = -6.0*m[np-2,i]/h[np-2]+6.0*m[np-3,i]/h[np-2]+(6.0*(-vf[i]*h[np]+1/3*h[np]**2*af[i]+m[np-1,i]))/h[np-1]-6.0*m[np-2,i]/h[np-1]
			capp[np-1] = -(-6.0*vf[i]*h[np]+2*h[np]**2*af[i]+6.0*m[np-1,i]-6.0*m[np-2,i]+3*h[np]*af[i]*h[np-1]-6.0*h[np-1]*vf[i])/h[np-1]
			c << capp
		end
		c = Matrix.rows(c).transpose
		f = (a.inv * c).to_a
		acc = [ai]
		for i in 0..np-1
			acc.push f[i]
		end
		acc.push af	
 		vp1 = [vi[0]*h[0]+1/3.0*h[0]**2*ai[0]+1/6.0*h[0]**2*acc[1][0]+m[0,0],
 			   vi[1]*h[0]+1/3.0*h[0]**2*ai[1]+1/6.0*h[0]**2*acc[1][1]+m[0,1],
 			   vi[2]*h[0]+1/3.0*h[0]**2*ai[2]+1/6.0*h[0]**2*acc[1][2]+m[0,2],
 			   vi[3]*h[0]+1/3.0*h[0]**2*ai[3]+1/6.0*h[0]**2*acc[1][3]+m[0,3]]
 		vp2 = [-vf[0]*h[np]+1/3.0*h[np]**2*af[0]+1/6.0*h[np]**2*acc[np][0]+m[np-1,0],
 			   -vf[1]*h[np]+1/3.0*h[np]**2*af[1]+1/6.0*h[np]**2*acc[np][1]+m[np-1,1],
 			   -vf[2]*h[np]+1/3.0*h[np]**2*af[2]+1/6.0*h[np]**2*acc[np][2]+m[np-1,2],
 			   -vf[3]*h[np]+1/3.0*h[np]**2*af[3]+1/6.0*h[np]**2*acc[np][3]+m[np-1,3]]
 		
 		q << m.row(0).to_a
 		q << vp1.push(tm[1])
 		for i in 1..np-2
 			q << m.row(i).to_a
 		end
 		q << vp2.push(tm[np])
 		q << m.row(np-1).to_a 		
 		
 		kmax = Array.new()
 		for n in 0..3
 			jc = 0 		
	 		j = Array.new()
			v = Array.new()
	 		l = Array.new()
	 		t = Array.new() 	
	 		ktot = 0.0		
 			for i in 0..np
		 		kmax[i] = ((tm[i+1]-tm[i])/tq).to_i
		 		#kmax[i] +=1 if ((tm[i+1]-tm[i])%tq)!=0.0
 				for k in 0..kmax[i]-1
 					t[jc+k] = tm[i]+k*tq
 					j[jc+k] = spline_pos(t[jc+k],tm[i],tm[i+1],h[i],q[i][n],q[i+1][n],acc[i][n],acc[i+1][n])
 					#printf("j[%i] = %f\n",k+jc,spline_pos(t[k],tm[i],tm[i+1],h[i],q[i][n],q[i+1][n],acc[n][i],acc[n][i+1]))
 					#gets
 					v[jc+k] = spline_vel(t[jc+k],tm[i],tm[i+1],h[i],q[i][n],q[i+1][n],acc[i][n],acc[i+1][n])
 					l[jc+k] = spline_acc(t[jc+k],tm[i],tm[i+1],h[i],acc[i][n],acc[i+1][n])
 					ktot +=1.0
 				end
		 		jc += kmax[i]
 			end		
 			joints  << j
 			vjoints << v
 			ajoints << l
 		end
 		optimized = true
 		k = 0
		ta = r.tavail
 		while k <= ktot-1 and optimized == true
 			r.joints  = [joints[0][k],joints[1][k],joints[2][k],joints[3][k]]
 			r.vjoints = [vjoints[0][k],vjoints[1][k],vjoints[2][k],vjoints[3][k]]
 			r.ajoints = [ajoints[0][k],ajoints[1][k],ajoints[2][k],ajoints[3][k]]
 			tr = r.dynamics("j")
 			dt = Array.new()
 			dv = Array.new()
 			[0,1,2,3].each do |i|
 				dt[i] = ta[i].abs - tr[i].abs
 			end
			#puts r.joints.inspect
			#puts r.vjoints.inspect
			#puts r.ajoints.inspect 			
 			#puts tr.inspect
 			#puts ta.inspect
 			#gets
 			[0,1,2,3].each do |i|
 				dv[i] = r.vmax[i]-r.vjoints[i].abs
 			end
 			#puts dt.min
 			#puts dv.min
 			#gets
 			if dt.min < 0 or dv.min < 0
 				optimized = false
 			else
 				k += 1
 			end
 		end	
 		if !optimized
 			ts = tm[0]+k*tq
 			k = 0
 			i = 0
			while k == 0
				k = i if m[i,4]-ts > 0.0	
				i += 1
 			end
 			mat = m.to_a
 			for i in k..np-1
 				mat[i][4] += dT
 			end
 			m = Matrix.rows(mat)
 		end
 		#gets
 		#printf(" -- %i -- \n",counter) 		
 		#printf ("tempi = [%f,%f,%f,%f]\n",m[0,4],m[1,4],m[2,4],m[3,4])
 		#puts optimized.inspect
 		#puts h.inspect
 		#puts m.to_a.inspect
 		#gets
 		if counter < 10
 			printf("    %i | %i | %f | %f | %f | %f\n",counter,k,tr[0].abs,tr[1].abs,tr[2].abs,tr[3].abs)
 		elsif counter < 100
 			printf("   %i | %i | %f | %f | %f | %f\n",counter,k,tr[0].abs,tr[1].abs,tr[2].abs,tr[3].abs)
 		else
 			printf("  %i | %i | %f | %f | %f | %f\n",counter,k,tr[0].abs,tr[1].abs,tr[2].abs,tr[3].abs)
 		end	
	end
	
	cj = Array.new()
	for k in 0..ktot-1
		cj << {:time => t[k], :joints => [joints[0][k],joints[1][k],joints[2][k],joints[3][k]]}
	end
	#puts cj.inspect
	return cj
end

private
	def	spline_acc(t,t1,t2,h1,f1,f2)
		return	(t2-t)/h1*f1+(t-t1)/h1*f2
	end
	
	def spline_vel(t,t1,t2,h1,q1,q2,f1,f2)
		return -(t2-t)**2/(2.0*h1)*f1+(t-t1)**2/(2.0*h1)*f2+q2/h1-h1*f2/6.0-q1/h1+h1*f1/6.0
	end
	
	def spline_pos(t,t1,t2,h1,q1,q2,f1,f2)
		return (t2-t)**3/(6.0*h1)*f1+(t-t1)**3/(6.0*h1)*f2+(t-t1)*(q2/h1-h1*f2/6.0)+(t2-t)*(q1/h1-h1*f1/6.0)
	end	

end

end

if __FILE__ == $0

end
