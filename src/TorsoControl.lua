require 'include.class'
require 'include.PID'

TorsoControl = class(function(self,name1,name2)
name1 = name1 or 'torso_lift_joint'
name2 = name2 or 'head_pan_link_visual'
self.TorsoJointHandle = sim.getObjectHandle(name1)
self.headHandle = sim.getObjectHandle(name2)
self.default_height = 1.2 -- head height
self.height_control = pid('torso')
self.height_control:set(0.5,0,0)
--self.height_control:set_max_min(1,0)

self.once_flag = false
end)

function TorsoControl:update()
  local position = sim.getObjectPosition(self.headHandle,-1)
  local err = self.default_height - position[3]
  local v_r =self.height_control:calculate(err)
  sim.setJointTargetVelocity(self.TorsoJointHandle,v_r)
 
end