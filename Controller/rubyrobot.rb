#!/usr/bin/env ruby

require "yaml"
require 'rubygems'
require 'serialport'
require "lib/coordinatesystem"
require "lib/ik"
require "lib/viewer"
require "lib/help"
require "lib/basicshape"
require "lib/arduino"

%w[ABRT HUP INT TERM].each do |s|
  Signal.trap(s) do
    running = false
	end
end

help = Help.new

if ARGV.include? "--help"
	help.general_help
	exit(0)	
end

include Arduino
include InverseKinematics

arduino = Controller.new("/dev/ttyUSB0",57600)

config = {
  :l => [0.0, 100.0,100.0, 10.0],
  :home => [  0.0,  0.0,  0.0],
  :limits => [
    -180.0.to_rad..180.0.to_rad,
    -180.0.to_rad..180.0.to_rad,
    -180.0.to_rad..180.0.to_rad,
    -180.0.to_rad..180.0.to_rad
  ]
}
r = Puma560.new(config)
target = {:x=>100.0, :y => 100.0, :z => -10.0, :phi => -90.0.to_rad}
r.ik(target)
joints = r.joints.map {|v| v.to_deg}

v = RobotViewer.new
v.bodies << BoxBody.new(
  :l => 0, 
  :a => 0, 
  :w => 10, 
  :theta => joints[0], 
  :alpha => 90.0, 
  :color => [1,1,1,0.9])
v.bodies << BoxBody.new(
  :l => 0, 
  :a => 100, 
  :theta => joints[1], 
  :alpha => 0.0,
  :color => [1.0,0.8,0.8,0.9])
v.bodies << BoxBody.new(
  :l => 0, 
  :a =>100, 
  :theta => joints[2], 
  :alpha => 0.0, 
  :color => [0.8,1.0,0.8,0.9])
v.bodies << BoxBody.new(
  :l => 0, 
  :a =>10,
  :w => 2, 
  :theta => joints[3], 
  :alpha => 0.0, 
  :color => [0.8,0.0,1.0,0.9])

server_thread = Thread.new { v.run }

if ARGV.include? "-circle"
t = 0
print "Do you want to change the Coordinate System? <y/n>: "
resp = STDIN.gets.chomp
if resp == "y"
	changeCS = true
	rtm = arduino.get_rtm(r,target,v)
else
	changeCS = false
	rtm = Matrix.identity(4)
end
	
end

if ARGV.include? "-datacapture"
	arduino.datacapture(r,target,v)
end

if ARGV.include? "-circle"
	include BasicShape
	puts "Creating path..."
	param = YAML::load_file("parameters.yaml")[:circle]
	circle = Circle.new
	circle_path = circle.create(param.merge({:rtm => rtm}))
	t  = 0.0
	dt = 0.05
	T  = 5.0
	points = Array.new()
	crosspoints = Array.new(0,Hash.new)
	while t <= T
		points << circle_path.call(t)	
		crosspoints << {:x => points.last[0],:y => points.last[1],
						:z => points.last[2],:phi => -1.57, :time => t}
		t += dt
		File.open("crosspoints.yaml", "w") {|f| YAML.dump(crosspoints, f)}
	end
end

if ARGV.include? "-auto" or ARGV.include? "-circle"
	filename = "crosspoints.yaml"
	arduino.automatic_mode(r,v,filename)
end
server_thread.join
server_thread.exit
