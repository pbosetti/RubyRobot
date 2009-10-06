require 'matrix'
require 'coordinatesystem'

module BasicShape

	class ConicalSpiral
		def create (p0 = Point.new([0.0,0.0,0.0]), vr = 1.0, vz = 5.0, 
                    omega = 10.0, ri = 10.0, zi=0.0, rtm = nil)
			p = Array.new(3)
			proc do |t|
				p1 = Matrix.column_vector [(ri+vr*t)*Math::cos(omega*t),
									  (ri+vr*t)*Math::sin(omega*t),
									  zi+vz*t,
									  1.0]
				p1 = rtm * p1
				p1.column(0).to_a[0..2]
			end			
		end
	end
	
	class CilindricalSpiral < ConicalSpiral
		def create (p0 = Point.new([0.0,0.0,0.0]), vz = 5.0, 
                    omega = 10.0, r = 10.0, zi=0.0, rtm = nil)
			super(p0 , 0.0, vz, omega, r, zi, rtm)
        end
	end
	
	class Spiral < ConicalSpiral
		def create (p0 = Point.new([0.0,0.0,0.0]), vr = 1.0, 
                    omega = 10.0, ri = 10.0, z=0.0, rtm = nil)
			super(p0 , vr, 0.0, omega, ri, z, rtm)
        end
	end

	class Circle < ConicalSpiral
		def create (p0 = Point.new([0.0,0.0,0.0]), 
                    omega = 10.0, r = 10.0, z=0.0, rtm = nil)
			super(p0 , 0.0, 0.0, omega, r, z, rtm)
        end
			
	end

end

if __FILE__ == $0
include CoordinateSystem
include BasicShape

p0 = Point.new([-1.0 , 0.0 , 1.0])
p1 = Point.new([1.0 , 2.0 , 1.0])
p2 = Point.new([0.0 , 1.0 , 1.0])

t1 = CartesianAxis.new(p0,p1,p2)
rtm = t1.rtm

r = 40
omega = 0.1
t = 0
T = 500

circle = Circle.new
circle_path = circle.create(Point.new([1.0,3.0,-2.0]),omega, r, 0.0, rtm)

conical_spiral = ConicalSpiral.new
conical_spiral_path = conical_spiral.create(Point.new([1.0,3.0,-2.0]), 1.0, 2.0, omega, 0.0, 0.0, rtm)

points = Array.new
x = Array.new
y = Array.new
z = Array.new
for t in 0..T
#	points << circle_path.call(t)	
	points << conical_spiral_path.call(t)
end

points.each_index do |i|
	x << points[i][0]
	y << points[i][1]
	z << points[i][2]		
end

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
