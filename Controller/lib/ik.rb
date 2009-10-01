class Float
  TO_RAD = Math::PI/180.0
  def to_rad; self * TO_RAD; end
  def to_deg; self / TO_RAD; end
end

module InverseKinematics
  include Math
  
  class OutOfRange < Exception

  end
  
  class Puma560
   attr_accessor :l, :home, :pose, :limits
    attr_accessor :joints
    attr_reader :wrist
    def initialize(cfg={})
      @l =      cfg[:l]     
      @home =   cfg[:theta] 
      @limits = cfg[:limits]
      @pose =   {:x => 0.0, :y => 0.0, :z => 0.0, :phi => 0.0}  
      @joints = Array.new(4,0.0)
    end
    
   def ik(pose=@pose)
      theta = atan2(pose[:y],pose[:x])
      r     = sqrt(pose[:x]**2+pose[:y]**2)
      sol = [Array.new(4,0), Array.new(4,0)]
      sphi = sin(pose[:phi])
      cphi = cos(pose[:phi])
      @wrist = [r-@l[3]*cphi, pose[:z]-@l[0]-@l[3]*sphi]
      c2 = (@wrist[0]**2+@wrist[1]**2-@l[1]**2-@l[2]**2)/(2*@l[1]*@l[2])
      s2 = sqrt(1-c2**2)
      k1 = @l[1] + @l[2]*c2
      k2 = @l[2]*s2
      sol[0][0] = theta
      sol[1][0] = theta
      sol[0][1] = atan2(@wrist[1], @wrist[0]) - atan2(k2, k1)
      sol[1][1] = atan2(@wrist[1], @wrist[0]) - atan2(-k2, k1)
      sol[0][2] = atan2(s2,c2)
      sol[1][2] = -sol[0][2]
      sol[0][3] = pose[:phi] - (sol[0][1] + sol[0][2])
      sol[1][3] = pose[:phi] - (sol[1][1] + sol[1][2])
      cumError = Array.new(2,0.0);
      [0,1,2,3].each do |i|
      	cumError[0] += (@joints[i]-sol[0][i])**2
      	cumError[1] += (@joints[i]-sol[1][i])**2      	
      end
      if cumError[1] > cumError[0]
      	if self.inlimits? sol[0]
      		@joints = sol[0]
      		@pose = pose
      		return 1
      	else
      		return 0
      	end	
      else	
      	if self.inlimits? sol[1]
      		@joints = sol[1]
      		@pose = pose
      		return 1
      	else
      		return 0
      	end
      end      	
      	  
    end
    
    def inlimits? (solution=@joints)
    	inlimits = true
    	solution.each_index do |i|
      		inlimits &= @limits[i].include? solution[i]
      	end
      	return inlimits
    end
  end
end  


if __FILE__ == $0  
include InverseKinematics
  config = {
  :l => [0.0, 100.0,100.0, 10.0],
  :home => [  0.0,  0.0,  0.0],
  :limits => [
    -90.0.to_rad..90.0.to_rad,
    0.0..85.0.to_rad,
    -90.0.to_rad..90.0.to_rad,
    -90.0.to_rad..90.0.to_rad
  ]
}
r = Puma560.new(config)
target = {:x=>100.0, :y => 100.0, :z => -10.0, :phi => -90.0.to_rad}
r.ik(target)
puts r.joints
end
