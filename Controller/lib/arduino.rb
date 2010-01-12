# Created by Fabiano Giuliani on 2009-05-25.
# Copyright (c) 2009 University of Trento. All rights reserved.

require 'rubygems'
require 'serialport'
require "lib/coordinatesystem"
require "lib/ik"

include CoordinateSystem

module Arduino

USBPORT  = "/dev/ttyUSB0"
BAUDRATE = 57600

SERVOS = {:base => 1, :shoulder => 2, :elbow => 3, :wrist=> 4} # servo number

class Controller

	def initialize(usbport = USBPORT, baudrate = BAUDRATE, arduino = true)
		@arduino = SerialPort.new(usbport, baudrate, 8, 1, SerialPort::NONE) if arduino 
	end
	
	def print (string)
		@arduino.print string
	end
	
	def puts (string)
		@arduino.puts string
	end
	
	def gets
		return @arduino.gets
	end
	
	def data_capture(r,target,v,filename)
		@arduino.print "M"
		first_time_click = 0.0
		running = true
		p = Array.new(1)
		line = [0,0,0,0,0]
		now = last_click = Time.now.to_f
		crosspoints = Array.new(0,Hash.new)
		crossjoints = Array.new(0,Hash.new)
		while running do
		  line = @arduino.gets.split
		  line = line.map {|e| e.to_i}
		  rebound = (Time.now.to_f - last_click < 1)
		  begin
		  	target[:x] -= line[1]/50000.0
		  	target[:y] += line[2]/50000.0
		   	target[:z] += line[3]/50000.0
			target[:phi] += line[4]/1000000.0
			if !r.ik(target)
				target[:x] += line[1]/50000.0
		  		target[:y] -= line[2]/50000.0
		   		target[:z] -= line[3]/50000.0
				target[:phi] -= line[4]/1000000.0
			end
			if line[0] == 1 and ! rebound
				if first_time_click == 0
					STDOUT.puts "------- Data capture begin --------"
		   			first_time_click = Time.now.to_f
		   	 		crosspoints[0] = {:time => 0}.merge(target)
		   	 		crossjoints[0] = {:time => 0, :joints => r.joints}
				else
					crosspoints << {:time => Time.now.to_f-first_time_click}.merge(target)
					crossjoints << {:time => Time.now.to_f-first_time_click, :joints => r.joints}
				end
				STDOUT.print r.joints, " "
			end			
			update_sim(r.joints,v)	
			
			if line[0] == 8 and ! rebound
				File.open(filename, "w") {|f| YAML.dump(crosspoints, f)}
				STDOUT.puts "------- Data capture end --------"
				running = false
			end
			if line[0] == 1 or line [0] == 8 and ! rebound
			  STDOUT.puts
			  last_click = Time.now.to_f
			end		
		  rescue
			STDOUT.puts "Error: #{$!} #{line.inspect}"
		  end
		end
	end
	
	def get_rtm (r,target,v)
		@arduino.print "M"
		first_time_click = 0.0
		running = true
		line = [0,0,0,0,0]
		now = last_click = Time.now.to_f
		p = Array.new(0)
		while running
			line = @arduino.gets.split
			line = line.map {|e| e.to_i}
			rebound = (Time.now.to_f - last_click < 1)
			begin
			  	target[:x] -= line[1]/50.0
			  	target[:y] += line[2]/50.0
			   	target[:z] += line[3]/50.0
				target[:phi] += line[4]/1000.0
				if !r.ik(target)
					target[:x] += line[1]/50.0
			  		target[:y] -= line[2]/50.0
			   		target[:z] -= line[3]/50.0
					target[:phi] -= line[4]/1000.0
				end
			#end
			if line[0] == 1 and ! rebound
				STDOUT.print "Point n°"
				STDOUT.print p.length+1
				STDOUT.print ": "
				STDOUT.print r.joints, " "
				STDOUT.puts
				if first_time_click == 0.0
					first_time_click =Time.now.to_f
					p[0] = Point.new([target[:x],target[:y],target[:z]])
					last_click = Time.now.to_f
				else
					p.push Point.new([target[:x],target[:y],target[:z]])
					last_click = Time.now.to_f
					if p.length == 3
					# cs.rtm is the Roto-Traslation matrix of this Coordinate System
						cs = CartesianAxis.new(p[0],p[1],p[2])
						rtm = cs.rtm
						running = false
					end	
				end	
			end
			update_sim(r.joints,v)
			rescue
				STDOUT.puts "Error: #{$!} #{line.inspect}"
		  	end
		end	
		return rtm
	end
	
	def automatic_mode(r,v,filename)
		STDOUT.print "Press <Enter> to begin the simulation."
		STDIN.gets
		@arduino.print "A" if @arduino # AUTOMATIC MODE
		sleep 1
		crossjoints = YAML::load_file(filename)
		last_time = 0.0
		time = 0.0
		#@arduino.print "S"
		crossjoints.each do |cj|
		t_ok = false
 		sthread = Thread.new { sleep cj[:time]-last_time; t_ok = true }		
			update_sim(cj[:joints],v)
			time += cj[:time]-last_time
			STDOUT.print time
			if @arduino
			for i in 0..3 do
				joint = (cj[:joints][i]*100.0).to_i				
				if joint < 0
				    #STDOUT.print((-joint/256+100).to_i)
					@arduino.print((-joint/256+100).to_i.chr)
					#STDOUT.print "+"
				else
					#STDOUT.print((joint/256).to_i) 
					@arduino.print((joint/256).to_i.chr)
					#STDOUT.print "+"
				end	
				#STDOUT.print((joint%256).to_i)
				@arduino.print((joint%256).to_i.chr)
				#STDOUT.print "\n"
			end
			end
			STDOUT.puts t_ok ? "+" : "-"
			last_time = cj[:time]
		sthread.join
		end
		@arduino.print "M" if @arduino
	end

private

	def update_sim(joints,v)
		v.bodies.each_with_index do |b, i|
			b.theta = joints[i].to_deg
		end
	end
	
end



end

if ($0 == __FILE__) 
	include Arduino
	
end




