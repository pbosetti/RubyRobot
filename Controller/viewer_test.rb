#!/usr/bin/env ruby
#
# Created by Paolo Bosetti on 2009-07-24.
# Copyright (c) 2009 University of Trento. All rights 
# reserved.
require "rubygems"
require "serialport"
require "lib/viewer"

%w[ABRT HUP INT TERM].each do |s|
  Signal.trap(s) do
    running = false
	end
end
port_file = Dir.glob("/dev/tty.usbserial*")[0]
sp = SerialPort.new(port_file, 57600, 8, 1, SerialPort::NONE)
puts "Connected with #{port_file}"

v = RobotViewer.new
v.bodies << BoxBody.new(
  :l => 0, 
  :a => 0, 
  :w => 10, 
  :theta => 30.0, 
  :alpha => 90.0, 
  :color => [1,1,1,0.9])
v.bodies << BoxBody.new(
  :l => 0, 
  :a => 50, 
  :theta => 45.0, 
  :alpha => 0.0,
  :color => [1.0,0.8,0.8,0.9])
v.bodies << BoxBody.new(
  :l => 0, 
  :a =>40, 
  :theta => -90.0, 
  :alpha => 0.0, 
  :color => [0.8,1.0,0.8,0.9])
v.bodies << BoxBody.new(
  :l => 0, 
  :a =>10,
  :w => 2, 
  :theta => -45.0, 
  :alpha => 0.0, 
  :color => [0.8,0.0,1.0,0.9])
server_thread = Thread.new { v.run }

running = true
line = [0,0,0,0,0]
now = last_click = Time.now.to_f

while running do
  line = sp.gets.split
  line = line.map {|e| e.to_i}
  rebound = (Time.now.to_f - last_click < 1)
  begin
    v.bodies.each_with_index do |b, i|
      b.theta += line[i+1]/100.0
      if line[0] == 1 and ! rebound
        print b.theta, " "
      end
    end
    if line[0] == 1 and ! rebound
      puts
      last_click = Time.now.to_f
    end
  rescue
    puts "Error reading serialport: #{line.inspect}"
  end
  # sleep 0.1
end
server_thread.join
