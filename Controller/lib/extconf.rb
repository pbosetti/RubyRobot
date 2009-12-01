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

equations = {:pose    => ["","","",""],
			 :vel	  => ["","","",""],
			 :acc	  => ["","","",""],
			 :vjoints => [["","","",""],["","","",""]],
			 :ajoints => [["","","",""],["","","",""]],
			 :tjoints => ["","","",""]}
			 
File.open('Equations.txt', "r") do |f|
	for i in 0..3
		string = f.gets.chomp
		equations[:pose][i] = string[8..string.size]
	end
	for i in 0..3
		string = f.gets.chomp
		equations[:vel][i] = string[8..string.size]
	end
	for i in 0..3
		string = f.gets.chomp
		equations[:acc][i] = string[8..string.size]
	end
	for i in 0..3
		string = f.gets.chomp
		equations[:vjoints][0][i] = string[8..string.size]
		if i!=0
			string = f.gets.chomp
			equations[:vjoints][1][i] = string[8..string.size]
		end	
	end
	for i in 0..3
		string = f.gets.chomp
		equations[:ajoints][0][i] = string[8..string.size]
		if i!=0
			string = f.gets.chomp
			equations[:ajoints][1][i] = string[8..string.size]
		end
	end
	for i in 0..3
		string = f.gets.chomp
		equations[:tjoints][i] = string[8..string.size]
	end
end
			 

File.open('dyneqns.c', "w") do |f| 
	f.puts eruby.result(binding()) 
end

create_makefile("Dynamics")
