#!/usr/bin/env ruby

require "yaml"
require 'rubygems'
require 'serialport'
require "lib/coordinatesystem"
#require "lib/ik"
require "lib/viewer"
require "lib/help"
require "lib/basicshape"
require "lib/arduino"
require "lib/Dynamics"

%w[ABRT HUP INT TERM].each do |s|
  Signal.trap(s) do
    running = false
	end
end

system "clear"
help = Help.new

if ARGV.include? "--help"
	help.general_help
	exit(0)	
end

include Arduino
include InverseKinematicsAndDynamics

if !ARGV.include? "-no-arduino"
begin
	baudrate = 57600
	usbport  = "/dev/ttyUSB0"
	arduino  = Controller.new(usbport,baudrate)
rescue
	warn "ERROR: No device connected."
	exit(0)
end
puts "Connected with #{usbport} @#{baudrate} bps"
end

basicShape = {:circle => "-circle", :conical_spiral => "-conicalspiral",
			  :cilindrical_spiral => "-cilindricalspiral",
			  :spiral => "-spiral"}

STDOUT.sync = true

# Parsing the ARGV array

shape = nil
datacapture = false
filename = ""
basicShape.each do |key,val|
	if ARGV.include? val
		if shape == nil
			shape = key
		else
			puts "ERROR: Too many information. Select #{basicShape[shape]} or #{basicShape[key]}"
			exit(0)
		end
	end	
end

if !shape and ARGV.include? "-datacapture"
	datacapture = true
end

ARGV.each do |v|
	filename = v if v.include? ".yaml"
end	

# End parsing

config = {
  :l => [0.0, 100.0,100.0, 10.0],
  :home => [  0.0,  0.0,  0.0],
  :limits => [
    -180.0.to_rad..180.0.to_rad,
    -180.0.to_rad..180.0.to_rad,
    -180.0.to_rad..180.0.to_rad,
    -180.0.to_rad..180.0.to_rad
  ],
  :psi => -90.0.to_rad,
  :a => [0.0, 0.0, 0.0, 0.0],
  :alpha => [0.0, 0.0, 0.0, 0.0],
  :d => [0.0, 0.0, 0.0, 0.0],  
  :m => [0.0, 0.0, 0.0, 0.0],
  :inertia => [[0.0, 0.0, 0.0],  # Ix1, Iy1, Iz1
			   [0.0, 0.0, 0.0],  # Ix2, Iy2, Iz2
			   [0.0, 0.0, 0.0],  # Ix3, Iy3, Iz3
			   [0.0, 0.0, 0.0]], # Ix4, Iy4, Iz4
  :mm => [0.0, 0.0, 0.0, 0.0],
  :Rext => [0.0, 0.0, 0.0, 0.0],
  :Text => [0.0, 0.0, 0.0, 0.0]  
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

if shape != nil
	t = 0
	resp = ""
	print "Do you want to change the Coordinate System? <y/n>: "
	resp = STDIN.gets.chomp
	puts
	if resp == "y"
		rtm = arduino.get_rtm(r,target,v)
	else
		rtm = Matrix.identity(4)
	end
		resp = ""
end

if datacapture
	arduino.data_capture(r,target,v,filename)
	print "Do you want to do the selected path? <y/n>: "
	resp = STDIN.gets.chomp
elsif shape != nil
	include BasicShape
	print "Creating path"
	param = YAML::load_file("parameters.yaml")[shape]
	case shape
		when :circle
			sh = Circle.new
		when :conical_spiral
			sh = ConicalSpiral.new
		when :cilindrical_spiral
			sh = CilindricalSpiral.new
		when :spiral
			sh = Spiral.new
	end
	path = sh.create(param.merge({:rtm => rtm}))
	t  = 0.0
	dt = 0.05
	T  = 5.0
	points = Array.new()
	crosspoints = Array.new(0,Hash.new)
	while t <= T
		points << path.call(t)	
		crosspoints << {:x => points.last[0],:y => points.last[1],
						:z => points.last[2],:phi => -1.57, :time => t}
		t += dt
		if t/T > 0.75
			print "."
		elsif t/T > 0.50
			print "."
		elsif t/T > 0.25
			print "."
		end	
		filename = "crosspoints.yaml" if filename == ""
		File.open(filename, "w") {|f| YAML.dump(crosspoints, f)}
	end
	print "finish"
	puts
	print "Do you want to do the selected path? <y/n>: "
	resp = STDIN.gets.chomp
end

if resp == "y"
	filename = "crosspoints.yaml" if filename == ""
	arduino.automatic_mode(r,v,filename)
end

server_thread.join
server_thread.kill
arduino.puts "M"
puts "Program ended - Click ENTER to exit"
STDIN.gets
