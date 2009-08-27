require "rubygems"
require "serialport"

running = true

begin
	port_file = "/dev/ttyUSB0"
	sp = SerialPort.new(port_file, 9600, 8, 1, SerialPort::NONE)
rescue
	warn "ERROR: No device connected."
	exit(0)
end	

iang = 20
fang = 100
time = 5000

sp.puts "#{iang}i"
sp.puts "#{fang}f"
sp.puts "#{time}t"
sp.puts "r"

sp.puts "s"
sleep 0.1

data = {:time => Array.new(),
		:theoric_angle => Array.new(),
		:measured_angle => Array.new()}

while running
	line = sp.gets
	if line != nil
		line = line.split
		data[:time] << line[0].to_f
		data[:theoric_angle] << line[1].to_f
		data[:measured_angle] << line[2].to_f/10.0
	end	
	running = false if line == nil
	sleep 0.05	
end

data.each {|d| d.delete_at(0)}

sp.puts "r"

File.open("servotest.r", "w") do |f| 
	f.puts "rm(list=ls(all=TRUE));"
	f.puts "split.screen(c(2,1));"
	f.puts "time <- c(#{data[:time].join(",")});"
	f.puts "theoric_angle <- c(#{data[:theoric_angle].join(",")});"
	f.puts "measured_angle <- c(#{data[:measured_angle].join(",")});"
	f.puts "screen(1);"	
	f.puts "plot(time,measured_angle, main=\"#{iang} -> #{fang} in #{time} ms\");"
	f.puts "screen(2);"
	f.puts "plot(theoric_angle,measured_angle, main=\"#{iang} -> #{fang} in #{time} ms\");"
	f.puts "df <- data.frame(time, R = measured_angle);"
	f.puts "df.lm <- lm(R ~ time^2, data = df);"
	f.puts "coefficients(df.lm)"
end

output = exec "R --vanilla < servotest.r"

puts output.inspect
