require 'include.class'
require 'include.PID'


MoveBase = class(function(self,name)
    name = name or 'fetch'
    self.MoveBaseHandle = sim.getObjectHandle(name)
    self.motorLeft=sim.getObjectHandle(name.."_leftMotor")
    self.motorRight=sim.getObjectHandle(name.."_rightMotor")
    self.lad = sim.getObjectHandle('look_ahead')
    sim.setObjectParent(self.lad,-1,true)
    self.__angular_controller = pid('angular controller ')
    self.__linear_controller = pid('linear controller')
--    self.__angular_controller:set(-0.5,0.05,0)
--    self.__linear_controller:set(10.5,10,0.0)
    self.__angular_controller:set(-2.5,-0.0,0.00)
    self.__linear_controller:set(2.5,0.0,0.0)
    self.__goal_position = sim.getObjectPosition(self.lad,-1)
    
 end)
 
function MoveBase:state(...)
  if(...)then
    local p = sim.getObjectPosition(self.lad,self.MoveBaseHandle)
    local theta = math.atan2(p[2],p[1])
    return {p[1], p[2],theta}
  else
    local p = sim.getObjectPosition(self.MoveBaseHandle,-1)
    local q = sim.getObjectOrientation(self.MoveBaseHandle, -1)
    return {p[1], p[2],math.pi/2 - q[2]}
  end
end


function MoveBase:update()
    local dx = self:state('lad')


    --the difficult solution requires coordinate transformation e.g.
--[[
    self.__goal_position = sim.getObjectPosition(self.lad,-1)
    local dx = self:dist_from_MoveBase(self.__goal_position)
--    ]]
    dx[1] = -dx[1]
    local dist = math.sqrt(math.pow(dx[1],2) + math.pow(dx[2],2))
    local orientation_err =  math.atan2(dx[2],dx[1])
--    orientation_err = math.fmod(orientation_err,1.0)
--    print(dist, orientation_err)
    -- MoveBase coordinate is at the center 
    -- but look ahead distance should be at front
    -- otherwise MoveBase will circle around to find its center
    --print(dist, dx)
    if(dist<0.135)then
      dist = 0
    end
     if(math.abs(orientation_err)<0.1)then
      orientation_err = 0
    end
    
--    print(dx,dist,orientation_err)
    local v = self.__linear_controller:calculate(dist)
    local w = self.__angular_controller:calculate(orientation_err)
    self:set_motor_speed(v, w)
    return dist
    
end

function MoveBase:goal()
  return  sim.getObjectPosition(self.lad,-1)
end

function MoveBase:set_goal(posi)
  posi[3]= 0.05
  sim.setObjectPosition(self.lad,-1,posi)
end

function MoveBase:diff_MoveBase_model(v_des, omega_des)
  local d, r_w, v_r, v_l, omega_right, omega_left
--    d = 1.5  --# wheel axis distance
--    r_w = 0.151  --# wheel radius
--    d = 0.5  --# wheel axis distance0.5
--    r_w = 0.09  --# wheel radius 0.05
    d = 0.5  --# wheel axis distance0.5
    r_w = 0.11 --# wheel radius 0.05
    v_r = (v_des + d * omega_des)
    v_l = (v_des - d * omega_des)
    omega_right = v_r / r_w
    omega_left = v_l / r_w
    return omega_right, omega_left

end

function MoveBase:set_motor_speed(v, w)
  local v_r, v_l
  v_r, v_l = self:diff_MoveBase_model(v,w)
--  local point = sim.getObjectPosition(self.lad,-1)
--  local dx = self:dist_from_MoveBase(point)
--   local dist = math.sqrt(math.pow(dx[1],2) + math.pow(dx[2],2))
--  if(dist<0.14)then 
--    print(dist)
--    v_r = 0
--    v_l = 0
--  end
  sim.setJointTargetVelocity(self.motorLeft,v_r)
  sim.setJointTargetVelocity(self.motorRight,v_l)
  --print(v_r,v_l)
end 




function MoveBase:dist_from_MoveBase(point)
--[[
  by defualt vrep provides transformation matrix of an object 
  a single table (vectpr). We need to separate 3x4 matrix out of it.
  Then we need separate 3x3 Rotational matrix and 3x1 Translation.
  However, we need transpose of Rotational matrix for calculating
  distance. 

]]
  local m = require 'include.matrix'
    local get_body_frame = function ()
      local mat = sim.getObjectMatrix(self.MoveBaseHandle,-1)
      local dim ={3,4}
      local transfrom = m:new(dim[1],dim[2],0)
      -- decode mat 
      for i = 1,dim[1] do
          for j = 1,dim[2] do 
              local index = j + (i-1)*dim[2]
              transfrom[i][j]=mat[index]
          end
          --print(transfrom[i])
      end
      -- obtain R and T
      local R = m:new(3,3)
      local T = m:new(3,1)
      for i = 1, 3 do 
          for j = 1,3 do 
              R[i][j] = transfrom[i][j]
          end
      end
      
      for k = 1, 3 do 
          T[k][1] = transfrom[k][4]
      end 
--     R = m.transpose(R)
     return R, T 
    end
    
    local R, T = get_body_frame()
    
    point[3] = T[3][1]
    point = m:new(point)
    local dT = m.sub( point, T)
    T = m.mul(R,dT)
    local res = {}
    for i = 1,3 do 
      res[i] = T[i][1]
    end
  return res
end
