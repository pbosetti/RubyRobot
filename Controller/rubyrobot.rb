#!/usr/bin/env ruby

require "rubygems"
require "serialport"
require "lib/viewer"
require "lib/ik"
require "yaml"
require "lib/coordinatesystem"
require "lib/help"

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

begin
	#port_file = Dir.glob("/dev/tty.usbserial*")[0]
	port_file = "/dev/ttyUSB0"
	sp = SerialPort.new(port_file, 57600, 8, 1, SerialPort::NONE)
rescue
	warn "ERROR: No device connected."
	exit(0)
end	
puts "Connected with #{port_file}"

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

crosspoints = Array.new(0,Hash.new)
crossjoints = Array.new(0,Hash.new)
continue    = ""
if ARGV.include? "-circle"
print "raggio ="
r = gets.to_f
print "omega ="
omega = gets.to_f
print "T ="
period = gets.to_f
t = 0
puts "Do you want to change the Coordinate System? <y/n> "
resp = gets
if resp == "y"
	
else

end
circle = Circle.new
circle_path = circle.create(Point.new([1.0,3.0,-2.0]),omega, r, 0.0, rtm)

end
if ARGV.include? "-datacapture" # MANUAL MODE
first_time_click = 0.0
running = true
changeCS = false
p = Array.new(1)
line = [0,0,0,0,0]
now = last_click = Time.now.to_f

while running do
  line = sp.gets.split
  line = line.map {|e| e.to_i}
  rebound = (Time.now.to_f - last_click < 1)
  begin
  	if line
  	target[:x] -= line[1]/100.0
  	target[:y] += line[2]/100.0
   	target[:z] += line[3]/100.0
	target[:phi] += line[4]/5000.0
    if !r.ik(target)
    	target[:x] += line[1]/100.0
  		target[:y] -= line[2]/100.0
   		target[:z] -= line[3]/100.0
		target[:phi] -= line[4]/5000.0
	end
	end
	if line[0] == 1 and ! rebound
		if changeCS
			include CoordinateSystem
			if first_time_click == 0
				first_time_click = Time.now.to_f								
				p[0] = Point.new([target[:x],target[:y],target[:z]])
			else
				p.push Point.new([target[:x],target[:y],target[:z]])
				last_click = Time.now.to_f
				if p.length == 3
				# cs.rtm is the Roto-Traslation matrix of this Coordinate System
					cs = CartesianAxis.new(p[0],p[1],p[2])
					puts cs.rtm.inspect
					changeCS = false
				end	
			end			
		else
			if first_time_click == 0
				puts "------- Data capture begin --------"
   		 		first_time_click = Time.now.to_f
   	 			crosspoints[0] = {:time => 0}.merge(target)
   	 			crossjoints[0] = {:time => 0, :joints => r.joints}
    		else
    			crosspoints << {:time => Time.now.to_f-first_time_click}.merge(target)
    			crossjoints << {:time => Time.now.to_f-first_time_click, :joints => r.joints}
    		end
    	end
    end
    v.bodies.each_with_index do |b, i|
      		b.theta = r.joints[i].to_deg
    if line[0] == 1 and ! rebound
        	print b.theta, " "
    end
    end
    
    if line[0] == 8 and ! rebound
	    File.open("crosspoints.yaml", "w") {|f| YAML.dump(crosspoints, f)}
	    File.open("crossjoints.yaml", "w") {|f| YAML.dump(crossjoints, f)}
	    puts "------- Data capture end --------"
	    running = false
    end
    if line[0] == 1 or line [0] == 8 and ! rebound
      puts
      last_click = Time.now.to_f
    end
# ------- Change coordinate system --------
	if line[0] == 6 and !rebound
		changeCS = true
	end    
    
  rescue
    puts "Error: #{$!} #{line.inspect}"
  end
  sleep 0.01
end
#server_thread.join
sleep 0.5
puts "Do you want to continue in automatic mode? <yes/no> :"
continue = gets
end

if continue == "yes" || continue == "y" || continue == ""
sp.puts "A" # AUTOMATIC MODE
puts "Automatic mode"

crosspoints = YAML.load_file("crosspoints.yaml")
crossjoints = YAML::load_file("crossjoints.yaml")

last_time = 0.0
crossjoints.each do |cj|
	v.bodies.each_with_index do |b, i|
      		b.theta = cj[:joints][i].to_deg
    end
    sleep cj[:time]-last_time
    last_time = cj[:time]
end
server_thread.join
end
server_thread.exit
