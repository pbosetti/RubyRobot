require 'matrix'

module CoordinateSystem

AXES = {:X => 0, :Y => 1, :Z => 2}
class Point < Array
    def Point.[](x=nil, y=nil, z=nil)
      Point.new([x, y, z])
    end
    
    def [](i)
      case i
      when Numeric
        i = i.to_i
      when Symbol
        i = AXES[i]
      when String
        i = AXES[i.to_sym]
      else
        raise ArgumentError
      end
      super(i)
    end
    
    def []=(i, v)
      case i
      when Numeric
        i = i.to_i
      when Symbol
        i = AXES[i]
      when String
        i = AXES[i.to_sym]
      else
        raise ArgumentError
      end
      super(i, v)
    end
    
    def r
      Math::sqrt(self[0]**2 + self[1]**2 + self[2]**2)
    end
    
    def normalize
    	self / self.r
    end
    
    def / (n)
    	self.each_index do |i|
    		self[i] /= n
    	end	
    end
    
    def -(other)
      Point[ self[0] - other[0],
        self[1] - other[1],
        self[2] - other[2]]
    end
    
    def inspect
      "[#{self[:X] or '-'} #{self[:Y] or '-'} #{self[:Z] or '-'}]"
    end
end


class CartesianAxis
    attr_reader :p0, :vx, :vy, :vz, :rtm, :plane
    
	def initialize (p0,p1,p2)
		@p0 = p0
		plane = Plane.new(p0,p1,p2)
		@vx = (p1-p0).normalize
		@vz = plane.ortho_vector(p0)
		@vy = Point.new([@vx[2]*@vz[1]-@vx[1]*@vz[2] , @vx[0]*@vz[2]-@vz[0]*@vx[2] , @vz[0]*@vx[1]-@vx[0]*@vz[1]]).normalize
		@rtm = Matrix.columns [@vx.push(0),@vy.push(0),@vz.push(0),@p0.push(1)]
	end

end

class Plane
	attr_reader :coeff
	
	def initialize(p1,p2,p3)
		@coeff = {:a => p2[1]*p3[2]-p2[1]*p1[2]-p1[1]*p3[2]-p2[2]*p3[1]+p2[2]*p1[1]+p1[2]*p3[1],
				  :b => -p2[0]*p3[2]+p2[0]*p1[2]-p3[0]*p1[2]+p3[0]*p2[2]+p1[0]*p3[2]-p1[0]*p2[2],
				  :c => -p3[0]*p2[1]+p2[0]*p3[1]+p1[0]*p2[1]-p2[0]*p1[1]+p3[0]*p1[1]-p1[0]*p3[1],
				  :d => p3[0]*p2[1]*p1[2]-p3[0]*p2[2]*p1[1]-p2[0]*p1[2]*p3[1]-p1[0]*p2[1]*p3[2]+p2[0]*p1[1]*p3[2]+p1[0]*p3[1]*p2[2] }	  
	end
	
	def ortho_vector(point)
		v = Point.new([point[0]+@coeff[:a],point[1]+@coeff[:b],point[2]+@coeff[:c]])
		return v.normalize
	end
	
	
end 

end

if __FILE__ == $0
require 'matrix'
require 'coordinatesystem'
include CoordinateSystem

p0 = Point.new([5.0 , 0.0 , -1.0])
p1 = Point.new([1.0 , 2.0 , -3.0])
p2 = Point.new([-1.0 , 1.0 , 1.0])

t1 = CartesianAxis.new(p0,p1,p2)
puts t1.rtm

end

