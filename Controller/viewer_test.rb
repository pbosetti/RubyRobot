#!/usr/bin/env ruby
#
# Created by Paolo Bosetti on 2009-07-24.
# Copyright (c) 2009 University of Trento. All rights 
# reserved.

require "lib/viewer"

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
45.times do |i|
  v.bodies.each do |b|
    b.theta += 1
  end
  sleep 0.1
end
server_thread.join
