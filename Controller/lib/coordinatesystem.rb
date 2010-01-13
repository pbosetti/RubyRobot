# Created by Fabiano Giuliani on 2009-05-25.
# Copyright (c) 2009 University of Trento. All rights reserved.

# = Operation with coordinate system
# This module manage the cartesian axis and calculate the roto-traslation matrix.

require 'matrix'

module CoordinateSystem

AXES = {:X => 0, :Y => 1, :Z => 2}

# The class Point inherit from Array.
class Point < Array

# Create a new istance of Point class.
    def Point.[](x=nil, y=nil, z=nil)
      Point.new([x, y, z])
    end

# Reader accessor to the array values.
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

# Writer accessor to the array values.    
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

# Calculate the phythagoric length of a vector.    
    def r
      Math::sqrt(self[0]**2 + self[1]**2 + self[2]**2)
    end

# Returns the normalized vector.
    def normalize
    	self / self.r
    end

# Divide every value of array to n.
    def / (n)
    	self.each_index do |i|
    		self[i] /= n
    	end	
    end

# Calculate the vector between two Point.    
    def -(other)
      Point[ self[0] - other[0],
        self[1] - other[1],
        self[2] - other[2]]
    end
    
    def inspect
      "[#{self[:X] or '-'} #{self[:Y] or '-'} #{self[:Z] or '-'}]"
    end
end

# The Cartesian Axis class calculate the roto-traslation matrix from 3 Points:
# - <b>p0</b> rapresent the origin point
# - <b>p1</b> rapresent the direction of x axes
# - <b>p0</b> rapresent a general point on xy plane.
class CartesianAxis

    attr_reader :p0, :vx, :vy, :vz, :rtm, :plane

# Returns a roto-traslation matrix of this CaertesianAxis class.
	def initialize (p0,p1,p2)
		@p0 = p0
		@plane = Plane.new(p0,p1,p2)
		@vx = (p1-p0).normalize
		@vz = plane.ortho_vector.normalize
		@vy = Point.new([@vx[2]*@vz[1]-@vx[1]*@vz[2] , @vx[0]*@vz[2]-@vz[0]*@vx[2] , @vz[0]*@vx[1]-@vx[0]*@vz[1]]).normalize
		@rtm = Matrix.columns [@vx.push(0),@vy.push(0),@vz.push(0),@p0.push(1)]
	end

end

# This class calculate the plane coefficients from 3 Points.
# The class variable <b>coeff</b> contain the coefficient a,b,c,d of equation ax+by+cz+d=0
class Plane
	attr_reader :coeff

# Returns the plane coefficients from 3 Points.	
	def initialize(p1,p2,p3)
		@coeff = {:a => p1[2]*p3[1]-p3[1]*p2[2]-p1[1]*p3[2]+p3[2]*p2[1]+p2[2]*p1[1]-p2[1]*p1[2],
				  :b => -p1[2]*p2[0]+p2[0]*p3[2]-p1[0]*p3[2]+p1[0]*p2[2]+p1[2]*p3[0]-p2[2]*p3[0],
				  :c => -p1[0]*p3[1]-p1[1]*p2[0]+p1[1]*p3[0]+p1[0]*p2[1]+p2[0]*p3[1]-p2[1]*p3[0],
				  :d => -p1[0]*p3[1]*p2[2]-p1[1]*p2[0]*p3[2]+p1[1]*p3[0]*p2[2]+p1[0]*p3[2]*p2[1]+p1[2]*p2[0]*p3[1]-p1[2]*p3[0]*p2[1]}	  
	end

# Returns the orthogonal vector of the Plane.
	def ortho_vector()
		v = Point.new([@coeff[:a],@coeff[:b],@coeff[:c]])
		return v.normalize
	end
	
	
end 

end

if __FILE__ == $0
require 'matrix'
require 'coordinatesystem'
include CoordinateSystem

p0 = Point.new([0.250 , 0.250 , -0.05])
p1 = Point.new([0.22 , 0.25 , -0.05])
p2 = Point.new([0.22 , 0.3 , -0.05])

t1 = CartesianAxis.new(p0,p1,p2)
puts t1.rtm

end

