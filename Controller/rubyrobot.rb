#!/usr/bin/env ruby
# Created by Fabiano Giuliani on 2009-10-05.
# Copyright (c) 2009 University of Trento. All rights reserved.

require "yaml"
require 'rubygems'
require 'serialport'
require 'optparse'

require "lib/coordinatesystem"
require "lib/viewer"
require "lib/basicshape"
require "lib/arduino"
require "lib/Dynamics"
require "lib/optimizer"

#%w[ABRT HUP INT TERM].each do |s|
#  Signal.trap(s) do
#    running = false
#	end
#end

options = {:automatic => true, :shape => nil, :filename => "", :optimized => true, :noarduino => false, :onlyruby => false}

opts = OptionParser.new do |opts|
  opts.on("-m","--manual filename", String, "\nAble the manual mode, capture the crosspoint when you press Joystick button 1 and end the data capture when you press the Joystick button 8. At the end of data capture create the yaml file (default crosspoints.yaml).\n") do |f|
  		options[:automatic] = false
		if f !=nil
			options[:filename] = f
		else
			options[:filename] = "crosspoints.yaml"
	   	end
	end
	opts.on("-c","--circle filename", String, "\nCreate a circolar path with the parameters specified in the parameters.yaml file and save the path in filename yaml file (default crosspoints.yaml).\n") do |f|
		options[:shape] = :circle
  		options[:automatic] = false
		if f !=nil
			options[:filename] = f
		else
			options[:filename] = "crosspoints.yaml"
	   	end
	end
	opts.on("-s","--spiral filename", String, "\nCreate a spiral path with the parameters specified in the parameters.yaml file and save the path in filename yaml file (default crosspoints.yaml).\n") do |f|
		options[:shape] = :spiral
  		options[:automatic] = false		
		if f !=nil
			options[:filename] = f
		else
			options[:filename] = "crosspoints.yaml"
	   	end
	end
	opts.on("-i","--cilindricalspiral filename", String, "\nCreate a cilindrical spiral path with the parameters specified in the parameters.yaml file and save the path in filename yaml file (default crosspoints.yaml).\n") do |f|
		options[:shape] = :cilindrical_spiral
  		options[:automatic] = false		
		if f !=nil
			options[:filename] = f
		else
			options[:filename] = "crosspoints.yaml"
	   	end
	end
	opts.on("-l","--conicalspiral filename", String, "\nCreate a conical spiral path with the parameters specified in the parameters.yaml file and save the path in filename yaml file (default crosspoints.yaml).\n") do |f|
		options[:shape] = :conical_spiral
  		options[:automatic] = false		
		if f !=nil
			options[:filename] = f
		else
			options[:filename] = "crosspoints.yaml"
	   	end
	end
	opts.on("-a","--automatic filename", String, "\nAble the automatic mode, send the coordinates of path to robot.\n") do |f|
		options[:automatic] = true
		if f !=nil
			options[:filename] = f
		else
			options[:filename] = "crosspoints.yaml"
	   	end
	end
	opts.on("-o", "--optimizer", "\nIf you want to optimize the path contained in the yaml file.\n") do |f|
		options[:optimized] = false
	end
	opts.on("-r", "--onlyruby", String, "\nIf you want to work with only the simulator, without the robot.\n") do |f|
		options[:onlyruby] = true
	end
	opts.on("-n", "--no-arduino", String, "\nIf you don't want to connect the Arduino microcontroller.\n") do |f|
		options[:noarduino] = true
	end
end
begin
	opts.parse! ARGV
rescue
	STDOUT.puts "Error: #{$!} "
	exit(0)
end

puts options.inspect
#gets

system "clear"

include Arduino
include InverseKinematicsAndDynamics
include Optimizer

begin
	baudrate = 57600
	usbport  = "/dev/ttyUSB0"
	arduino  = Controller.new(usbport,baudrate,!options[:noarduino])
rescue
	begin
		baudrate = 57600
		usbport  = "/dev/ttyUSB1"
		arduino  = Controller.new(usbport,baudrate,!options[:noarduino])
	rescue
		warn "ERROR: No device connected."
		exit(0)
	end
end
puts "Connected with #{usbport} @#{baudrate} bps"

if options[:onlyruby]
	arduino.print "R"
end

STDOUT.sync = true

config = {
  :l => [0.0, 0.25 , 0.25, 0.05], #[m]
  :home => {:x=>0.25, :y => 0.25, :z => -0.05, :phi => -90.0.to_rad},
  :limits => [
    20.0.to_rad..160.0.to_rad,
    10.0.to_rad..150.0.to_rad,
    -135.0.to_rad..-1.0.to_rad,
    -120.0.to_rad..20.0.to_rad
  ],
  :psi => -90.0.to_rad,
  :vmax => [7.80773041, 7.587469858, 10.471975513, 7.801621757], #[rad/s]
  :tmax => [1.29, 2.44, 0.78, 0.13], # [Nm]
  :m => [0.755, 0.0660, 0.0660, 0.0660],   # [kg]
  :inertia => [[0.0, 0.0, 0.0],  				 # Ix1, Iy1, Iz1 [kg m2]
  			   [0.0000149, 0.000344, 0.000344],  # Ix2, Iy2, Iz2 [kg m2]
  			   [0.0000149, 0.000344, 0.000344],  # Ix3, Iy3, Iz3 [kg m2]
  			   [298.0*10**(-6), 274.0*10**(-6), 274.0*10**(-6)]], # Ix4, Iy4, Iz4 [kg m2]
  :mm => [0.2, 0.2, 0.15, 0.10], #[kg]
  :Rext => [0.0, 0.0, 0.0, 0.0],
  :Text => [0.0, 0.0, 0.0, 0.0]  
}
r  = Puma560.new(config)
r2 = Puma560.new(config)

ai = [0,0,0,0]
af = [0,0,0,0]
vi = [0,0,0,0]
vf = [0,0,0,0]
tq = 0.01
dT = 0.05

target = config[:home]
r.ik(target)
r2.ik(target)
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
  :a => r.l[1]*1000, 
  :theta => joints[1], 
  :alpha => 0.0,
  :color => [1.0,0.8,0.8,0.9])
v.bodies << BoxBody.new(
  :l => 0, 
  :a =>r.l[2]*1000, 
  :theta => joints[2], 
  :alpha => 0.0, 
  :color => [0.8,1.0,0.8,0.9])
v.bodies << BoxBody.new(
  :l => 0, 
  :a =>r.l[3]*1000,
  #:w => 2, 
  :theta => joints[3], 
  :alpha => 0.0, 
  :color => [0.8,0.0,1.0,0.9])

server_thread = Thread.new { v.run }

if options[:shape] != nil
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
	include BasicShape
	print "Creating path"
	param = YAML::load_file("parameters.yaml")[options[:shape]]
	case options[:shape]
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
	dt = 0.1
	T  = 10.0
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
		File.open(options[:filename], "w") {|f| YAML.dump(crosspoints, f)}
	end
	
	cubicspline = PPOcubicspline.new(r2,options[:filename],vi,ai,vf,af,tq,dT)
	
	#STDOUT.print "finish"
	puts
	print "Do you want to do the selected path? <y/n>: "
	resp = STDIN.gets.chomp
	if resp == "y"
		options[:automatic] = true
		options[:filename] = cubicspline.optfn
	end
end	
if !options[:automatic]
	arduino.data_capture(r,target,v,options[:filename])
	cubicspline = PPOcubicspline.new(r2,options[:filename],vi,ai,vf,af,tq,dT)
	print "Do you want to do the selected path in automatic mode? <y/n>: "
	resp = STDIN.gets.chomp
	if resp == "y"
		options[:automatic] = true
		options[:filename] = cubicspline.optfn
		resp = ""
	end
end	

if options[:automatic]
	if !options[:optimized]
		cubicspline = PPOcubicspline.new(r2,options[:filename],vi,ai,vf,af,tq,dT)
		options[:filename] = cubicspline.optfn
	end
	arduino.automatic_mode(r,v,options[:filename])
end

#server_thread.join
server_thread.kill
puts "Program ended - Click ENTER to exit"
STDIN.gets
