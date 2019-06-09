moveBase = {
    motorLeft=sim.getObjectHandle("l_wheel_joint"),
    motorRight=sim.getObjectHandle("r_wheel_joint"),
    fetch = sim.getObjectHandle('fetch'),
    head_pan = sim.getObjectHandle('head_pan_joint'),
    head_pan_visual = sim.getObjectHandle('head_pan_link_visual'),
    head_tilt = sim.getObjectHandle('head_tilt_joint')
}

function moveBase:transform(traj)
    local scale = 1.5
    local x = 0
    local y = 0
    local newTraj = {}
    for i = 1,#traj do 
        x = x + self._origin[1] + traj[i][1] * 1.5
        y = y + self._origin[2] + traj[i][2]  * 1.0
        table.insert(newTraj,{x,y})
    end
    print(newTraj)
    return newTraj
end

function moveBase:getPolarCoord(setX,setY)
    local X,x,y,dx,dy
    X = sim.getObjectPosition(self.fetch,-1)
    x = X[1]
    y = X[2]
    dx = setX - x
    dy = setY - y
    rho = math.sqrt(x*x + y*y)
    alpha = math.atan2(dy,dx)
    signY = dy/math.abs(dy)
    signX = dx/math.abs(dx)
    return rho,alpha,signY,signX

end

function moveBase:getSetRadius()
    local X,dx,dy,setX,setY
    X = sim.getObjectPosition(self.fetch,-1)
    dx = 1.65
    dy = 0
    setX = X[1] + dx
    setY = X[2] + dy
    return math.sqrt(setX*setX + setY*setY)
end

function moveBase:getSetAngle()
    -- right (0,1)
    -- left math.pi-math.atan2(0,-1)
    return math.atan2(0,1)
end

function moveBase:new(o)
    o = o or {}
    setmetatable(o,self)
    self.__index = self
    self._origin = sim.getObjectPosition(self.fetch,-1)
    require 'pid@controller'
    self.linear = pid('linear')
    self.angular = pid('angular')
    self.linear:print()
    self.angular:print()
    -- self.linear = pid:new(nil,'right ')
    self.linear:set(5,0.8,0.00)
    self.angular:set(5,0.8,0.00)
    self.setRho = self:getSetRadius()
    self.setAlpha = self:getSetAngle()
    self.finshed = false
    return o
end

function moveBase:moveForward()
    local X,x,y,rho,v,finish
    X = sim.getObjectPosition(self.fetch,-1)
    x = X[1]
    y = X[2]
    rho = math.sqrt(x*x + y*y)
    v = self.linear:calculate(rho, self.setRho )
    finish = (v*100)<1.0
    -- print(self.setRho ,rho, v)

    return finish, v, v
end

function moveBase:compute(setX,setY)

    if(self.finshed ) then
        return 0,0
    end
    local rho, alpha, sign
    rho, alpha,signY,signX = moveBase:getPolarCoord(setX,setY)

    Q = sim.getObjectOrientation(self.fetch,-1)
    setRho = math.sqrt(setX*setX + setY*setY)
   
    angle = Q[2]*signY
    if(angle*signY>0) then
        angle = math.pi - angle
    end
    -- print(signX)
    if(signX<0) then
        v = self.linear:calculate(setRho, rho)
    else
        v = self.linear:calculate(rho, setRho)
    end
    w = self.angular:calculate( alpha*signY, angle)
    -- print(rho,alpha, v, w, sign)
    if(w>0) then
        if(signY<0) then     
            return  0 , w  
        else
            return  w , 0
        end  
    end

    print(v)
   
    self.finshed = v<=0.0
   
    return v,v
   
end

function moveBase:calculate()

    local rho, alpha
    rho, alpha = self:getPolarCoord()
    -- local X,x,y,rho,v,finish
    -- X = sim.getObjectOrientation(self.fetch,-1)
    -- alpha = X[2]
    -- -- self.setAlpha =0.0
    -- v = self.angular:calculate(self.setAlpha,alpha )
    -- print(alpha, self.setAlpha,v)
    -- finish = (v*100)<1.0
    return 0,0
end


-- function moveBase:calculate()
--     local finish, mLeft,mRight
--     finish, mLeft,mRight = self:moveForward()
--     if(finish)then
--         print('finished')
--     end
--     return mLeft,mRight
-- end

function moveBase:update(x,y)
  mRight,mLeft =moveBase:compute(x,y)
  sim.setJointTargetVelocity(self.motorLeft,mLeft)
  sim.setJointTargetVelocity(self.motorRight,mRight)

  local sum = mRight+mLeft
  return sum == 0

-- Q = sim.getObjectOrientation(self.head_pan_visual,-1)
-- if(math.abs(Q[2])>0.1) then
--   sim.setJointTargetVelocity(self.head_pan,-0.00)
-- else
--     sim.setJointTargetVelocity(self.head_pan,0.0)
-- end
--   print(Q)
--   sim.setJointTargetVelocity(self.head_tilt,10.0)
end