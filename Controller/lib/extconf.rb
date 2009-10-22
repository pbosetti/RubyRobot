#!/usr/bin/env ruby
#  extconf
#
#  Created by Paolo Bosetti on 2009-10-09.
#  Copyright (c) 2009 University of Trento. All rights reserved.
#

require 'mkmf'
require 'rubygems'
require 'erubis'

input = File.read('dyneqns.erubis')
eruby = Erubis::Eruby.new(input)

equations = {:vjoints => ["1","1","1","1"],
			 :ajoints => ["1","1","1","1"],
			 :tjoints  => ["1","1","1","1"]}
			 
#File.open('equations.txt', "r") do |f| 
#	for i in 0..3
#		string = f.gets.chomp
#		equations[:vjoints][i] = string[8..string.size]	
#	end
#	for i in 0..3
#		string = f.gets.chomp
#		equations[:ajoints][i] = string[8..string.size]	
#	end
#	for i in 0..3
#		string = f.gets.chomp
#		equations[:tjoints][i] = string[8..string.size]	
#	end
#end
			 

File.open('dyneqns.c', "w") do |f| 
	f.puts eruby.result(binding()) 
end

create_makefile("Dynamics")
