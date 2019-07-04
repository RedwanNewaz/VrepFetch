require 'include.class'
m = require 'include.matrix'


ObstacleDetector = class (function(self,name)
  name = name or 'LaserScanner_2D_2D_SCANNER_DATA'
     self.communicationTube=sim.tubeOpen(0,name,1)
     self.safety_dist = 3
     self.resolution = 15
     self.scanningAngle = 120
     self.scanningDensity = 2
     
    local range = self.scanningAngle/2 
    self.maxPossibleHit = self.scanningDensity * self.resolution
    self.check ={}
    self.sensorArray = {}
    for q = -range, range, self.resolution do 
        table.insert(self.check,q)
        table.insert(self.sensorArray,0)
    end
     
     
end)
function ObstacleDetector:sensorArrayUpdate(angles)
   local value = {}
   for i = 1, #self.sensorArray do 
      value[i] = 0
   end
    
    for i, theta in pairs(angles) do 
      -- check theta belongs to which group
      for j=1, #self.check do 
          if(theta<=self.check[j]) then
              value[j] = value[j] + 1
              break
          end    
      end    
    end
    for i, v in pairs(value) do 
      value[i] = v
      self.sensorArray[i] = v/self.maxPossibleHit
    end 
    return value
end


function ObstacleDetector:polarTransform(point)
  local dx = point
  local rho = math.sqrt( math.pow(dx[2],2) + math.pow(dx[3],2))
  local theta =  math.atan2(dx[3],dx[2])
  return rho, theta
end

function ObstacleDetector:update()

     local data=sim.tubeRead(self.communicationTube)
     if (data) then
         local laserDetectedPoints=sim.unpackFloatTable(data)
         local data = self:reshapePoints(laserDetectedPoints)
         local dim = m.size(data)
         
         local count = 0
         local _angle = {}
         for i = 1, dim do 
            local rho, theta = self:polarTransform(data[i])
            if(rho<self.safety_dist)then 
                 count = count +1
                 _angle[count] = (theta)*57.2958
            end
         end
         if(count>0)then
            local d = self:sensorArrayUpdate(_angle)
           -- print(d)
         end

     end

end


function ObstacleDetector:reshapePoints(points)
  local ind2sub = function(ind, array_shape)
      local rows, cols
      rows = math.floor((ind-1) / array_shape[2])+1
      cols = math.fmod((ind-1) , array_shape[2]) +1
      return rows, cols
  end
  local col = 3
  local row = (#points/col)
  local data = m:new(row,col,0)
  local array_shape = {row,col}
  for index, p in pairs(points)do
    local i, j = ind2sub(index, array_shape)
    data[i][j] = p
  end
 return data
end


--print(ind2sub(4,{182,3}))