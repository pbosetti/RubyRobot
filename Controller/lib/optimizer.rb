require 'gsl'
require 'matrix'

include GSL
include GSL::MultiMin
include Math

module Optimizer

class Optimizer

	def PPOcubicspline(r,filename,vi,ai,vf,af,tq,dT)
		type = "j"
		crosspoints = YAML::load_file(filename)
		crossjoints = Array.new(0,Hash.new)
		crosspoints.each do |cp|
			cj = cp.delete(:time)
		  	puts "Out of range!" if !r.ik(cp)
			crossjoints << r.joints.join << cj
		end
		np = crossjoints.length
		# caricare velocità e coppie massime
		m    = Matrix.rows(crossjoints)
		h    = Array.new(np+1)
		tm   = Array.new(np+2)
		a 	 = Matrix.zero(np)
		c	 = Array.new()
		capp = Array.new(np)
		q    = Array.new()
		acc  = Array.new()
		ajoints = Array.new()
		vjoints = Array.new()
		joints  = Array.new()
		
	while !optimized
		h[0] = (m[1,4]-m[0,4])/2
		h[1] = h[0]
		for i in 1..np-3
			h[i+1] = m[i+1,4]-m[i,4]
		end
		h[np-1] = (m[np-1,4]-m[np-2,4])/2
		h[np] 	= h[np-1]
		tm[0]   = h[0]
		for i in 1..np+1
			tm[i] = tm[i-1]+h[i]
		end
		a[0,0] = 3*h[0]+2*h[1]+h[0]**2/h[1]
		a[0,1] = h[1]
		a[1,0] = h[1]-h[0]**2/h[1]
		a[1,1] = 2*(h[1]+h[2])
		a[1,2] = h[2]
		for i in 2..np-3
			a[i,i-1] = h[i]
			a[i,i] 	 = 2*(h[i]+h[i+1])
			a[i,i+1] = h[i+1]
		end
		a[np-2,np-3] = h[np-2]
		a[np-2,np-2] = 2*(h[np-2]+h[np-1])
		a[np-2,np-1] = h[np-1]-h[np]**2/h[np-1]
		a[np-1,np-2] = h[np-1]
		a[np-1,np-1] = 2*h[np-1]+h[np]**2/h[np-1]+3*h[np]
		
		for i in 0..3
			capp[0] = 6*(m[1,i]/h[1]+m[0,i]/h[0])-6*(1/h[0]+1/h[1])*(m[0,i]+h[0]*vi[i]+h[0]**2/3*ai[i])-h[0]*ai[i]
			capp[1] = 6/h[1]*(m[0,i]+h[0]*vi[i]+h[0]**2/3*ai[i])+6*m[2,i]/h[2]-6*(1/h[1]+1/h[2])*m[1,i]
			for n in 2..np-3
				capp[n] = 6*((m[n+1,i]-m[n,i])/h[n+1]-(m[n,i]-m[n-1,i])/h[n])
			end
			capp[np-2] = -6*m[np-2,i]/h[np-2]+6*m[np-3,i]/h[np-2]+(6*(-vf[i]*h[np]+1/3*h[np]**2*af[i]+m[np-1,i]))/h[np-1]-6*m[np-2,i]/h[np-1]
			capp[np-1] = -(-6*vf[i]*h[np]+2*h[np]**2*af[i]+6*m[np-1,i]-6*m[np-2,i]+3*h[np]*af[i]*h[np-1]-6*h[np-1]*vf[i])/h[np-1]
			c << capp
		end
		c = Matrix.rows(c).transpose
		f = (a.inv * c).to_a
 		acc = [(f[0].insert(0,ai[0])).push(af[0]),
 			   (f[1].insert(0,ai[1])).push(af[1]),
 			   (f[2].insert(0,ai[2])).push(af[2]),
 			   (f[3].insert(0,ai[3])).push(af[3])]
 		vp1 = [vi[0]*h[0]+1/3*h[0]**2*ai[0]+1/6*h[0]**2*acc[1][0]+m[0,0],
 			   vi[1]*h[0]+1/3*h[0]**2*ai[1]+1/6*h[0]**2*acc[1][1]+m[0,1],
 			   vi[2]*h[0]+1/3*h[0]**2*ai[2]+1/6*h[0]**2*acc[1][2]+m[0,2],
 			   vi[3]*h[0]+1/3*h[0]**2*ai[3]+1/6*h[0]**2*acc[1][3]+m[0,3]]
 		vp2 = [-vf[0]*h[np]+1/3*h[np]**2*af[0]+1/6*h[np]**2*acc[np][0]+m[np,0],
 			   -vf[1]*h[np]+1/3*h[np]**2*af[1]+1/6*h[np]**2*acc[np][1]+m[np,1],
 			   -vf[2]*h[np]+1/3*h[np]**2*af[2]+1/6*h[np]**2*acc[np][2]+m[np,2],
 			   -vf[3]*h[np]+1/3*h[np]**2*af[3]+1/6*h[np]**2*acc[np][3]+m[np,3]]
 		
 		q << m.row(0).to_a
 		q << vp1 << tm[1]
 		for i in 1..np-2
 			q << m.row(i)
 		end
 		q << vp2 << tm[np]
 		q << m.row(np-1).to_a 		
 		
 		j = Array.new()
		v = Array.new()
 		l = Array.new()		
 		
 		kmax = (tm[i+1]-tm[i])/tq.to_i
 		for n in 0..3
 			for i in 0..np
 				for k in 0..kmax-1
 					t = tm[i]+k*tq
 					j[k] = spline_pos(t,tm[i],tm[i+1],h[i],q[i,n],q[i+1,n],acc[i][n],acc[i+1][n])
 					v[k] = spline_vel(t,tm[i],tm[i+1],h[i],q[i,n],q[i+1,n],acc[i][n],acc[i+1][n])
 					l[k] = spline_acc(t,tm[i],tm[i+1],h[i],acc[i][n],acc[i+1][n])
 				end
 			end
 			joints  << j
 			vjoints << v
 			ajoints << l
 		end	   
 		optimized = true
 		k = 0
 		while k <= kmax-1 and optimized == true
 			r.joints  = [joints[0][k],joints[1][k],joints[2][k],joints[3][k]]
 			r.vjoints = [vjoints[0][k],vjoints[1][k],vjoints[2][k],vjoints[3][k]]
 			r.ajoints = [vjoints[0][k],vjoints[1][k],vjoints[2][k],vjoints[3][k]]
 			tr = r.dynamics(type)
 			ta = r.tavail
 			[0,1,2,3].each do |i|
 				dt = ta[i].abs - tr[i].abs
 			end
 			[0,1,2,3].each do |i|
 				dv = r.vmax[i]-r.vjoints[i].abs
 			end
 			if dt.min < 0 or dv.min < 0
 				optimized = false
 			else
 				k += 1
 			end
 		end
 		if !optimized
 			t = tm[0]+k*tq
 			k = 0
 			i = 0
			while k == 0
				k = i if tm[i]-t > 0
 			end
 			for i in k..np-1
 				m[i,4] += dT
 			end
 		end
 		
	end
		
	end
	
end

private
	def	spline_acc(t,t1,t2,h1,f1,f2)
		return	(t2-t)/h1*f1+(t-t1)/h1*f2
	end
	
	def spline_vel(t,t1,t2,h1,q1,q2,f1,f2)
		return -(t2-t)**2/(2*h1)*f1+(t-t1)**2/(2*h1)*f2+q2/h1-h1*f2/6-q1/h1+h1*f1/6
	end
	
	def spline_vel(t,t1,t2,h1,q1,q2,f1,f2)
		return (t2-t)**3/(6*h1)*f1+(t-t1)**3/(6*h1)*f2+(t-t1)*(q2/h1-h1*f2/6)+(t2-t)*(q1/h1-h1*f1/6)
	end	

end

if __FILE__ == $0
np = 2

my_f = Proc.new { |v, params|
  x = v[0]; y = v[1]
  p0 = params[0]; p1 = params[1]
  10.0*(x - p0)*(x - p0) + 20.0*(y - p1)*(y - p1) + 30.0
}

my_func = Function.alloc(my_f, np)
my_func.set_params([1.0, 2.0])      # parameters

x = Vector.alloc([5, 7])
ss = Vector.alloc(np)
ss.set_all(1.0)

minimizer = FMinimizer.alloc("nmsimplex", np)
minimizer.set(my_func, x, ss)

iter = 0
begin
  iter += 1
  status = minimizer.iterate()
  status = minimizer.test_size(1e-2)
  if status == GSL::SUCCESS
    puts("converged to minimum at")
  end
  x = minimizer.x
  printf("%5d ", iter);
  for i in 0...np do
    printf("%10.3e ", x[i])
  end
  printf("f() = %7.3f size = %.3f\n", minimizer.fval, minimizer.size);
end while status == GSL::CONTINUE and iter < 100

end
