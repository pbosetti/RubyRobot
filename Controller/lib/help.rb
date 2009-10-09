class Help

def initialize()

end

def general_help()
puts "---------- RubyRobot HELP ----------"
puts "ruby rubyrobot.rb [OPTIONS]"
puts
puts "OPTIONS: -datacapture => able the manual mode, capture the crosspoint when you
                         press Joystick button 1 and end the data capture when
                         you press the Joystick button 8. At the end of data
                         capture create the file crosspoint.yaml"
puts "         -[shapetype] => select the type of shape between \'circle\', \'spiral\'"
puts "                         \'conicalspiral\', \'cilindricalspiral\'."
puts "                         The parameters of this shape are loading from the file"
puts "                         parameters.yaml."
puts
puts "Help created by Giuliani Fabiano"							   
end

end
