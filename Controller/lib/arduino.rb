# Created by Fabiano Giuliani on 2009-05-25.
# Copyright (c) 2009 University of Trento. All rights reserved.

require 'rubygems'
require 'serialport'
require 'rubitlash'

module ARDUINO

USBPORT  = "/dev/ttyUSB0"
BAUDRATE = 9600

SERVOS = {:base => 0, :shoulder => 1, :elbow => 2, :wrist=> 3}

class Controller < SerialPort	

	def initialize(usbport = USBPORT, baudrate = BAUDRATE)
		begin
			super(usbport,baudrate)
		rescue
			raise "Arduino not connected"
		end
	end
end

class ServoMotor
	
	def initialize(controller = nil, position = :base ,theta = 0, config = {})
		@controller = controller
		@position   = position
		@number     = SERVOS[@position]
		@config		= config
		@profiler	= Profiler.new(@config)
		self.write(theta)
	end
	
	def write (value)
		
	end
	
	def read()
		
	end

	def inspect()
		if (@controller!=nil)
			puts "Servo on #{@position.to_s} (number #{@number}) connected on #{@controller}"
		else
			puts "Servo on #{@position.to_s} (number #{@number}) NOT connected!!"
		end
	end	
	
	def create_profile(w_s, w_m, w_e, theta)
		return @profiler.velocity_profile(w_s, w_m, w_e, theta)
	end
end

class Profiler
	
	attr_reader :times, :feeds, :accel, :dt
    
	def initialize(c={})
      @cfg = c
	end
    
	def velocity_profile(w_s, w_m, w_e, theta)
		w_s = w_s.to_f
		w_m = w_m.to_f
		w_e = w_e.to_f
		theta = theta.to_f

		dt_1 = ((w_m - w_s) / @cfg[:A]).abs
		dt_2 = ((w_m - w_e) / @cfg[:D]).abs
		dt_m = theta / w_m - dt_1 * (w_m+w_s)/(2*w_m) - dt_2 * (w_m+w_e)/(2*w_m)
      
	if dt_m >= 0.0
		q  = quantize(dt_1+dt_m+dt_2, @cfg[:t_q], :up)
		dt_m += q[1]
		w_m = (2*theta -w_s*dt_1 -w_e*dt_2)/(dt_1+dt_2+2*dt_m)
		a   = (w_m - w_s) / dt_1
		d   = (w_e - w_m) / dt_2
	else
		dt_1 = Math::sqrt(2*theta / (@cfg[:A] + @cfg[:A]**2 / @cfg[:D]))
		dt_2 = dt_1 * @cfg[:A] / @cfg[:D]
		q  = quantize(dt_1 + dt_2, @cfg[:t_q], :up)
		dt_m = 0.0
		dt_2 += q[1]
		w_m = w_s + 2 * theta / (dt_1 + dt_2)
		a = (w_m - w_s) / dt_1 
		d = (w_e - w_m) / dt_2
	end
      
	@times = [dt_1, dt_m, dt_2]
	@feeds = [w_s, w_m, w_e]
	@accel = [a, d]
	@dt    = q[0]

	proc do |t|
		r = 0
		if t < dt_1
			r = w_s * t + a * t**2 / 2
		elsif t < dt_1+dt_m
			r = (w_s + w_m) * dt_1 / 2 + w_m * (t - dt_1)
		else
			t_2 = dt_1 + dt_m
			r = (w_s + w_m) * dt_1 / 2 + w_m*dt_m + w_m*(t-t_2) + d/2 * (t**2 + t_2**2) - d*t*t_2
		end
		r
	end
end
    
private
    
	def quantize(x, q, dir = :up)
		r = []
		q = q.to_f
		if (x % q) == 0.0
			return [x, 0.0]
		end
      
	if dir == :up
		r[0] = ((x / q).to_i + 1) * q
		r[1] = r[0] - x
	else
		r << ((x / q). to_i) * q
		r << r[0] - x
	end
	r
    end

end

end

