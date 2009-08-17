#!/usr/bin/env ruby
#
# Created by Paolo Bosetti on 2009-07-24.
# Copyright (c) 2009 University of Trento. All rights 
# reserved.
require "rubygems"
require "serialport"
require "lib/viewer"
require "lib/ik"

%w[ABRT HUP INT TERM].each do |s|
  Signal.trap(s) do
    running = false
	end
end
port_file = Dir.glob("/dev/tty.usbserial*")[0]
sp = SerialPort.new(port_file, 57600, 8, 1, SerialPort::NONE)
puts "Connected with #{port_file}"

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
joints = r.joints[1].map {|v| v.to_deg}

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

running = true
line = [0,0,0,0,0]
now = last_click = Time.now.to_f

while running do
  line = sp.gets.split
  line = line.map {|e| e.to_i}
  rebound = (Time.now.to_f - last_click < 1)
  begin
    target[:x] -= line[1]/100.0
    target[:y] += line[2]/100.0
    target[:z] += line[3]/100.0
    target[:phi] += line[4]/5000.0
    r.ik(target)
    v.bodies.each_with_index do |b, i|
      b.theta = r.joints[1][i].to_deg
      if line[0] == 1 and ! rebound
        print b.theta, " "
      end
    end
    if line[0] == 1 and ! rebound
      puts
      last_click = Time.now.to_f
    end
  rescue
    puts "Error: #{$!} #{line.inspect}"
  end
  sleep 0.01
end
server_thread.join
