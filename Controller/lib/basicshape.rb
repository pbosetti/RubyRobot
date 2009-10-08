module BasicShape

	class ConicalSpiral
		def create (par={})
			@vr = par[:vr]
			@vz = par[:vz]
			@omega = par[:omega]
			@ri = par[:ri]
			@zi = par[:zi]
			@rtm = par[:rtm]
			p = Array.new(3)
			proc do |t|
				p1 = Matrix.column_vector [(@ri+@vr*t)*Math::cos(@omega*t),
									  	   (@ri+@vr*t)*Math::sin(@omega*t),
											@zi+@vz*t,
									  		1.0]
				p1 = @rtm * p1
				p1.column(0).to_a[0..2]
			end			
		end
	end
	
	class CilindricalSpiral < ConicalSpiral
		def create (par={})
			super(par.merge({:vr => 0.0}))
        end
	end
	
	class Spiral < ConicalSpiral
		def create (par={})
			super(par.merge({:vz => 0.0}))
        end
	end

	class Circle < ConicalSpiral
		def create (par={})
			super(par.merge({:vz => 0.0, :vr => 0.0, }))
        end			
	end

end

if __FILE__ == $0
require "yaml"
require "coordinatesystem"
include CoordinateSystem
include BasicShape

p0 = Point.new([100.0 , 100.0 , -10.0])
p1 = Point.new([10.0 , 0.0 , -10.0])
p2 = Point.new([0.0 , -1.0 , -10.0])

t1 = CartesianAxis.new(p0,p1,p2)
rtm = t1.rtm
zaxes = Point.new(rtm.column(2).to_a)
#phi = Math::acos(zaxes[2]/zaxes.r)
phi = -Math::acos(-1)/2


param_circle = {:omega => 5.0, :ri => 40.0, :zi => 0.0, :rtm => rtm}  
circle = Circle.new
circle_path = circle.create(param_circle)

param_conical_spiral = {:omega => 5.0, :ri => 10.0, :vr => 5.0,  :zi => 0.0,
						:vz => -10.0, :rtm => rtm}  
conical_spiral = ConicalSpiral.new
conical_spiral_path = conical_spiral.create(param_conical_spiral)

points = Array.new()
crosspoints = Array.new(0,Hash.new)
x = Array.new
y = Array.new
z = Array.new

t     = 0.0
dt    = 0.05
T     = 5.0

while t <= T
#	points << circle_path.call(t)	
	points << conical_spiral_path.call(t)
	crosspoints << {:x => points.last[0],:y => points.last[1],:z => points.last[2],
	             :phi => phi, :time => t}
	t += dt
end

points.each_index do |i|
	x << points[i][0]
	y << points[i][1]
	z << points[i][2]		
end

File.open("crosspoints.yaml", "w") {|f| YAML.dump(crosspoints, f)}

File.open("basicshape.r", "w") do |f|
	f.puts "library(rgl)"
	f.puts "rm(list=ls(all=TRUE));"
	f.puts "x <- c(#{x.join(",")})"
	f.puts "y <- c(#{y.join(",")})"
	f.puts "z <- c(#{z.join(",")})"	
	f.puts "plot3d(x,y,z,col=\"blue\")"
	f.puts "rgl.postscript(\"shape.pdf\", fmt=\"pdf\", drawText=TRUE)"
end

exec "R --vanilla -q < basicshape.r"

end
