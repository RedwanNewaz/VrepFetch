require 'include.class'
m = require 'include.matrix'

local ind2sub = function(ind, array_shape)
    local rows, cols
    rows = math.floor((ind-1) / array_shape[2])+1
    cols = math.fmod((ind-1) , array_shape[2]) +1
    return rows, cols
end
 local dist =function(dx)
  return  math.sqrt(math.pow(dx[1],2) + math.pow(dx[2],2))
 end
  
local reshapePoints = function(points)
  local col = 3
  local row = (#points/col)
  local data = m:new(row,col,0)
  local array_shape = {row,col}
  for index, p in pairs(points)do
    local i, j = ind2sub(index, array_shape)
    data[i][j] = p
  end

  --print(math.max(unpack(ii)), math.min(unpack(ii)), math.max(unpack(jj)), math.min(unpack(jj)))
  return data
end

ObstacleDetector = class (function(self,name)
  name = name or 'LaserScanner_2D_2D_SCANNER_DATA'
     self.communicationTube=sim.tubeOpen(0,name,1)
     self.safety_dist = 3
     self.laserHandle = sim.getObjectHandle('LaserScanner_2D')
end)

function ObstacleDetector:robotPosition()
  return sim.getObjectPosition(self.laserHandle, -1)
end

function ObstacleDetector:onBoundary(point)
local d = dist(point)+dist(self:robotPosition())
  return d>4.95
end

function ObstacleDetector:update()

     local data=sim.tubeRead(self.communicationTube)
     if (data) then
         laserDetectedPoints=sim.unpackFloatTable(data)
         local data = reshapePoints(laserDetectedPoints)
         local dim = m.size(data)
         
         local count = 0
         local boundary = false
         for i = 1, dim do 
            local d = dist(data[i])
            if(d<self.safety_dist)then 
            --[[
                local bb = self:onBoundary(data[i])
                if(bb) then 
                  boundary = true
                  else
                  count = count +1
                end
                ]]
                 count = count +1
            end
         end
         if(count>0)then
            print(count)
         end
   
     end

end


--print(ind2sub(4,{182,3}))