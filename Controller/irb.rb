require 'lib/Dynamics.so'
require 'lib/optimizer'

include InverseKinematicsAndDynamics
include Optimizer

config = {
  :l => [0.25, 0.25 ,0.25, 0.25],
  :home => [  0.0,  0.0,  0.0],
  :limits => [
    -180.0.to_rad..180.0.to_rad,
    -180.0.to_rad..180.0.to_rad,
    -180.0.to_rad..180.0.to_rad,
    -180.0.to_rad..180.0.to_rad
  ],
  :psi => -90.0.to_rad,
  :vmax => [6.977393246, 6.977393246, 8.716478191, 8.716478191], # inserire parametri caratterizzazione
  :tmax => [2.5, 2.5, 0.53955, 0.53955], # inserire parametri caratterizzazione   
  :m => [0.7550, 0.0660, 0.0660, 0.0660],
  :inertia => [[0.0019, 0.0030, 0.0030],  # Ix1, Iy1, Iz1
  			   [0.0, 0.0, 0.0014],  # Ix2, Iy2, Iz2
  			   [0.0, 0.0, 0.0014],  # Ix3, Iy3, Iz3
  			   [0.0, 0.0, 0.0014]], # Ix4, Iy4, Iz4
  :mm => [0.2, 0.2, 0.15, 0.10],
  :Rext => [0.0, 0.0, 0.0, 0.0],
  :Text => [0.0, 0.0, 0.0, 0.0]  
}
r = Puma560.new(config)
target = {:x=>0.25, :y => 0.25, :z => -0.25, :phi => -90.0.to_rad}
r.ik(target)
ai = [0,0,0,0]
af = [0,0,0,0]
vi = [0,0,0,0]
vf = [0,0,0,0]
tq = 0.05
dT = 0.05
filename = "optimizerpoints.yaml"
crossjoints = PPOcubicspline.new(r,filename,vi,ai,vf,af,tq,dT)
#r.dynamics("c")
