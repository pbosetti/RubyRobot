#!/usr/bin/env ruby
#
# Created by Paolo Bosetti on 2009-07-20.
# Copyright (c) 2009 University of Trento. All rights 
# reserved.
class Float
  TO_RAD = Math::PI/180.0
  def to_rad; self * TO_RAD; end
  def to_deg; self / TO_RAD; end
end

module RubyRobot
  include Math
  
  class OutOfRange < Exception

  end
  
  class Robot
    attr_accessor :l, :home, :pose, :limits
    attr_accessor :joints
    attr_reader :wrist
    def initialize(cfg={})
      @l =      cfg[:l]     
      @home =   cfg[:theta] 
      @limits = cfg[:limits]
      @pose =   {:x => 0.0, :y => 0.0, :z => 0.0, :phi => 0.0}  
      @joints = Array.new(3,0.0)
    end
  end
  
  class Planar3R < Robot
    def ik(pose=@pose)
      sol = [Array.new(3,0), Array.new(3,0)]
      @pose = pose
      sphi = sin(@pose[:phi])
      cphi = cos(@pose[:phi])
      @wrist = [@pose[:x]-@l[2]*cphi, @pose[:z]-@l[2]*sphi]
      c2 = (@wrist[0]**2+@wrist[1]**2-@l[0]**2-@l[1]**2)/(2*@l[0]*@l[1])
      s2 = sqrt(1-c2**2)
      sol[0][1] = atan2(s2, c2)
      sol[1][1] = -sol[0][1]
      k1 = @l[0] + @l[1]*c2
      k2 = @l[1]*s2
      sol[0][0] = atan2(@wrist[1], @wrist[0]) - atan2(k2, k1)
      sol[1][0] = atan2(@wrist[1], @wrist[0]) - atan2(-k2, k1)
      sol[0][2] = @pose[:phi] - (sol[0][0] + sol[0][1])
      sol[1][2] = @pose[:phi] - (sol[1][0] + sol[1][1])
      @joints = sol
    end
  end
  
  class Puma560 < Planar3R
    def ik(pose=@pose)
      @pose = pose
      theta = atan2(@pose[:y],@pose[:x])
      r     = sqrt(pose[:x]**2+pose[:y]**2)
      super({:x=> r, :z => @pose[:z], :phi => @pose[:phi]})
      @wrist = [@wrist[0]*cos(theta),@wrist[0]*sin(theta),@wrist[1]]
      @joints[0].insert 0, theta
      @joints[1].insert 0, theta
      [0,1,2,3].each do |i|
      #	warn "Out of range!" if !@limits[i].include? @joints[0][i]
      #	warn "Out of range!" if !@limits[i].include? @joints[1][i]      	
     raise OutOfRange if !@limits[i].include? @joints[1][i]      	
      end
      @joints
    end
  end
end

if ($0 == __FILE__) 
  include RubyRobot
  config = {
    :l => [100.0,100.0, 10.0],
    :home => [  0.0,  0.0,  0.0],
    :limits => [
      [0.0, 85.0.to_rad],
      [-90.0.to_rad, 90.0.to_rad],
      [-90.0.to_rad, 90.0.to_rad]
    ]
  }
  r = Puma560.new(config)
  target = {:x=>100.0, :y => 100.0, :z => -10.0, :phi => -90.0.to_rad}
  r.ik(target)
  puts "Inverse kinematics test for #{r.class} robot"
  puts "target: [#{target[:x]},#{target[:y]},#{target[:z]}] angle #{target[:phi].to_deg} (deg)"
  puts "\nsolution 1:", r.joints[0].map {|v| v.to_deg}.inspect
  puts "\nsolution 2:", r.joints[1].map {|v| v.to_deg}.inspect
  puts "\nwrist coordinates:", r.wrist.inspect
end
