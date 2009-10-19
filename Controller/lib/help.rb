#= This class explains the options of this software.
class Help

def initialize()

end

# Writes in the command windows the general option.
def general_help()
puts "---------- RubyRobot HELP ----------"
puts "ruby rubyrobot.rb [OPTIONS] [filename.yaml]"
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
puts "[filename.yaml]: the file will contains the coordinates of selected path"
puts 
puts "Help created by Giuliani Fabiano"							   
end

end
