# Created by Fabiano Giuliani on 2009-05-25.
# Copyright (c) 2009 University of Trento. All rights reserved.

require 'rubygems'
require 'serialport'

module ARDUINO

USBPORT  = "/dev/ttyUSB0"
BAUDRATE = 57600

SERVOS = {:base => 1, :shoulder => 2, :elbow => 3, :wrist=> 4} # servo number

class Controller < SerialPort	

	def initialize(usbport = USBPORT, baudrate = BAUDRATE)
		begin
			super(usbport,baudrate, 8, 1, SerialPort::NONE)
		rescue
			raise "Arduino not connected"
		end
	end
end

class ServoMotor
	attr_accessor :profiler
	def initialize(controller = nil, position = :base ,theta = 0, config = {})
		@controller = controller
		@position   = position
		@number     = SERVOS[@position]
		@config		= config
		self.write(theta)
	end
	
	# il metodo write invia sulla porta seriale 3 byte
	# 1° byte = codice ascii della lettera w moltiplicato per il servo da comandare
	# 2° e 3° byte = valore dell'angolo in gradi moltiplicato per 100
	#                con risoluzione alla seconda cifra decimale
	def write (value)
		if (@controller!=nil)
			data = (?> * @number).chr
			firstByte  = (value/127).to_i.chr
			secondByte = (value%127).to_i.chr
  			data.insert(1,firstByte)
  			data.insert(2,secondByte)
			@controller.write data
		end
	end
	
	def read()
		val = ""
		if (@controller!=nil)
			data = (?< * @number).chr
			@controller.write data
		end
		while val==""
			val = @controller.read
			puts "Valore " + val if val!=""
		end			
	end

	def inspect()
		if (@controller!=nil)
			puts "Servo on #{@position.to_s} (number #{@number}) connected on #{@controller}"
		else
			puts "Servo on #{@position.to_s} (number #{@number}) NOT connected!!"
		end
	end	
	
end

end

if ($0 == __FILE__) 
	include ARDUINO
	configServo = {
	  :A => 50,
	  :D => 60,
	  :t_q => 0.05
	}
	
	configStepper = {
	  :A => 50,
	  :D => 60,
	  :t_q => 0.005,  
	  :theta_q => 1.8
	}

	arduino_diecimila = Controller.new(usb="/dev/ttyUSB0", baud=9600)
	servo1 = ServoMotor.new(arduino_diecimila, :base, 0, configServo)
	servo2 = ServoMotor.new(arduino_diecimila, :shoulder, 0, configServo)
	servo1.inspect
	servo2.inspect

	time  = 0
	dt    = configServo[:t_q]
	feed  = 60
	theta = 180
	profile = servo1.profiler.velocity_profile(0, feed, 0, theta)
	value = profile.call(time)	
	sleep 1
	while value < 360
		puts value
		servo1.write value*100
		servo2.write value*100
		sleep dt
		time += dt
		value = profile.call(time)
	end
	
	#servo1.read
	puts "--------------- Working done -----------------"
end




