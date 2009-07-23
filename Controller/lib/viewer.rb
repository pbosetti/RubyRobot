#!/usr/bin/env ruby
#
# Created by Paolo Bosetti on 2009-03-30.
# Copyright (c) 2009 University of Trento. All rights 
# reserved.

require 'rubygems'
require 'opengl'
require 'mathn'
require 'drb'
include Gl,Glu,Glut

SCALE_FACTOR = 0.1

class BoxBody
  attr_accessor :l, :a, :alpha, :theta
  def initialize(pars={})
    @a = pars[:a] 
    @w = pars[:w] || @a/10.0
    @l = pars[:l] || 0.0
    @alpha = pars[:alpha] || 0.0
    @theta = pars[:theta] || 0.0
    @color = pars[:color] || [1,1,1,0.9]
  end
  
  def draw
    a2 = @a/2.0
    glColor(*@color)
    glRotate(@theta, 0.0, 0.0, 1.0)
    glTranslate(0.0, 0.0, @l)
    glTranslate(a2, 0.0, 0.0)
    glPushMatrix()
      glScale(@a == 0.0 ? @w: @a, @w, @w)
      glutWireCube(1.0)
    glPopMatrix()
    glTranslate(a2, 0.0, 0.0)
    glRotate(@alpha, 1.0, 0.0, 0.0)
  end
end

class RobotViewer
  TITLE = "Robot Viewer"
  attr_accessor :refresh_rate, :bodies, :axes_length
  attr_accessor :draw
  attr_accessor :x_rot, :y_rot, :z_rot
  
  def RobotViewer.open
    RobotViewer.new.run
  end
  
  def initialize(args={})
    @axes_length = 100
    @x_rot = 45
    @y_rot = -135
    @z_rot = 0
    @x_last = -1
    @y_last = -1
    @x_incr = 0
    @y_incr = 0
    @scale_factor = 5.0
    @modifiers = 0
    @draw = {:axes => true }
    
    @x_pan = 0.0
    @y_pan = 0.0
        
    @bodies = []
    @title  = args[:title] || TITLE
    
    glutInit
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_ALPHA)
    glutInitWindowSize(500, 500) 
    glutInitWindowPosition(100, 100)
    glutCreateWindow(@title)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glShadeModel     GL_SMOOTH
    glEnable         GL_DEPTH_TEST
    glEnable         GL_BLEND
    glEnable         GL_LINE_SMOOTH
    glBlendFunc      GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA
    glutDisplayFunc  self.display 
    glutReshapeFunc  self.reshape 
    glutKeyboardFunc self.keyboard
    glutMotionFunc   self.motion  
    glutMouseFunc    self.mouse  
    glutIdleFunc     self.idle 
    glutSpecialFunc  self.special
    @w, @h = glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT)
  end
  
  def status
    "#{@x_last}, #{@y_last}"
  end
  
  def run
    glutMainLoop()
    self
  end
  
  def display
    lambda do 
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_ACCUM_BUFFER_BIT)
      glPushMatrix()
      glRotate(@x_rot, 1,0,0)
      glRotate(@y_rot, 0,1,0)
      glRotate(@z_rot, 0,0,1)
      glScale(@scale_factor, @scale_factor, @scale_factor)
      
      glPushMatrix()         # Begin rotation 90° about X
      glRotate(-90, 1,0,0)   # so to have Z axis pointing upward
      glPushMatrix()
      @bodies.each_with_index do |b,i|
        b.draw
        # if @draw[:lines]
        #   if i < points.length - 1
        #     glColor(1,1,1,1)
        #     glBegin(GL_LINES)
        #       glVertex(points[i])
        #       glVertex(points[i+1])
        #     glEnd()
        #   end
        # end
        # if @draw[:points]
        #   glPushMatrix()
        #   glTranslate(*p) 
        #   glColor(1,0,0,0.9)
        #   glutSolidSphere(@point_size/@scale_factor, 16, 16)
        #   glPopMatrix()
        # end
      end
      glPopMatrix()
      if @draw[:axes]
        glLineWidth(1.0)
        glColor(1,0,0,0.66)
        glBegin(GL_LINES)
          glVertex(0,0,0)
          glVertex(@axes_length/@scale_factor,0,0)
        glEnd()
        glPushMatrix()
        glTranslate(@axes_length/@scale_factor,0,0)
        glRotate(90,0,1,0)
        glutSolidCone(20/@scale_factor, 30/@scale_factor, 20, 1)
        glPopMatrix()
        
        glColor(0,1,0,0.66)
        glBegin(GL_LINES)
          glVertex(0,0,0)
          glVertex(0,@axes_length/@scale_factor,0)
        glEnd()
        glPushMatrix()
        glTranslate(0,@axes_length/@scale_factor,0)
        glRotate(-90,1,0,0)
        glutSolidCone(20/@scale_factor, 30/@scale_factor, 20, 1)
        glPopMatrix()
        
        glColor(0,0,1,0.66)
        glBegin(GL_LINES)
          glVertex(0,0,0)
          glVertex(0,0,@axes_length/@scale_factor)
        glEnd()
        glPushMatrix()
        glTranslate(0,0,@axes_length/@scale_factor)
        glRotate(0,0,1,0)
        glutSolidCone(20/@scale_factor, 30/@scale_factor, 20, 1)
        glPopMatrix()
      end
      
      glPopMatrix()  # end rotation about X
      glPopMatrix()
      glFlush()
      glutSwapBuffers()
    end
  end
  
  def reshape
    lambda do |w,h|
      glViewport(0, 0,  w,  h) 
      glMatrixMode(GL_PROJECTION)
      glLoadIdentity()
      gluPerspective(65.0,  w/ h, 100.0, 5000.0)
      glMatrixMode(GL_MODELVIEW)
      glLoadIdentity()
      glTranslate(0, 0, -1200)
      @w, @h = glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT)
    end
  end
  
  def keyboard
    lambda do |key, x, y|
      case key
      when ?a
        @draw[:axes] = !@draw[:axes]
      when ?+
        @point_size *= 2.0
      when ?-
        @point_size /= 2.0
      when ?\e
        exit(0)
      else
        puts <<-EOM
Keystrokes:
  a           toggles axes visibility
  +           increases points size
  -           decreases points size
  Up/Down     rotate about Y (green)
  Left/Right  rotate about Z (blue)
  Home        reset rotation and scale
  ESC         exit
        EOM
      end
    end
  end
  
  def special
    special = lambda do |key,x,y|
      case key
      when GLUT_KEY_HOME
        @x_rot = 45
        @y_rot = -135
        @z_rot = 0
        @x_incr = 0
        @y_incr = 0
        @scale_factor = 1.0
      when GLUT_KEY_LEFT
        @y_rot -= 10
      when GLUT_KEY_RIGHT
        @y_rot += 10
      when GLUT_KEY_UP
        @z_rot += 10
      when GLUT_KEY_DOWN
        @z_rot -= 10
      end
    end
  end
  
  def motion
    lambda do |x, y|
      if (@x_last != -1 || @y_last != -1)
        @x_incr = x - @x_last
        @y_incr = y - @y_last
        if (@modifiers & GLUT_ACTIVE_CTRL != 0)
          if (@x_last != -1)
            @scale_factor += @y_incr * SCALE_FACTOR
            @scale_factor = @scale_factor.abs
            @x_pan = @y_pan = 0
          end
        elsif (@modifiers & GLUT_ACTIVE_SHIFT != 0)
          if (@x_last != -1)
            @x_pan = @x_incr*2.5
            @y_pan = -@y_incr*2.5
            glMatrixMode(GL_PROJECTION)
            glTranslate(@x_pan, @y_pan, 0.0)
            glMatrixMode(GL_MODELVIEW)
          end
        else
          if (@x_last != -1)
            @x_rot += @y_incr
            @y_rot += @x_incr
            @x_pan = @y_pan = 0
          end
        end
      end
      @x_last = x
      @y_last = y
    end
  end
  
  def mouse
    lambda do |button, state, x, y|
      @modifiers = glutGetModifiers()
      if (button == GLUT_LEFT_BUTTON)
        if (state == GLUT_UP)
          @x_last = -1
          @y_last = -1
        end
        @x_incr = 0
        @y_incr = 0
      end
    end
  end
  
  def idle
    lambda do
      glutPostRedisplay()
    end
  end  
  
end

if __FILE__ == $0
  # server = RobotViewer.new
  # server_thread = Thread.new { server.run }
  # DRb.start_service('druby://localhost:9000', server) 
  # DRb.thread.join
  # server_thread.kill
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
  v.run
end