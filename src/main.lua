
function simDrawLine(pt0,pt1,...)
    local data={pt0[1],pt0[2],0.01,pt1[1],pt1[2],0.01}
    local lineSize=5 -- in points
    local maximumLines=9999
    local color= ... or {1.,1.0,0.0}
    local drawingContainer=simAddDrawingObject(sim_drawing_lines,lineSize,0,-1,maximumLines,color) -- adds a line
    simAddDrawingObjectItem(drawingContainer,data)
end

function simShowGrid(m,n)
    local grid = require 'GridWorld'
    local map_size ={m,n}
    local block_size ={-2.3,2.7,0,2.54}
    grid.drawCells(block_size,map_size,simDrawLine)
end

function simShowTrajectory(...)
  local robot = {-2.0113048553467, -0.5750207901001}
  local traj = {{0,1},{1,0},{1,0}}
  local res = {}
  local x = robot 
  local y = robot
  local scaleX = 1.5
  local scaleY = 1.2 
  for i = 1, #traj do 
    res[i] = {y[1]+traj[i][1]*scaleX,y[2]+traj[i][2]*scaleY}
    y = res[i]
  end
  
  if(...) then
    -- visualize lines 
    local blue={0.,0.0,1.0}
    local setPoints ={}
    for i = 1, #traj do 
      setPoints[i] = {x,res[i]}
      simDrawLine(x,res[i],blue)
      x = res[i]
    end 
  end
  
  return res

end

function simGetState(objectHandle)
  require 'include.State'
  local X = sim.getObjectPosition(objectHandle,-1)
  local Q = sim.getObjectOrientation(objectHandle,-1)
  local C = {X,Q}
  setmetatable(C,State)
  return C()
end

function simGetWaypoints(setPoints, objectHandle)
  local X = simGetState(objectHandle)
  local waypoints = {}
  for i, v in pairs(setPoints) do 
    local dx = v[1] - X[1]
    local dy = v[2] - X[2]
    local psi = math.atan2(dy,dx) 
    waypoints[i] = {v[1],v[2],psi}
    X = {v[1],v[2],psi}
  
  end 
  return waypoints

end

function simGeneratePath(path)
    local setContorlPoint = function(pos)
        local ptData={pos[1],pos[2],pos[3],0.0,0.0,0.0,1.0,0,0,1.0,1.0}
        return ptData
    end
    local line_size = 5
    local intParams = {line_size,sim.distcalcmethod_dl,0}
    local pathHandle=sim.createPath(-1,intParams,nil,nil)
    print(sim.getObjectName(pathHandle))
    for i, data in pairs(path) do 
      local point = path[#path-i+1]
      point[3]= 0.05
      local result=sim.insertPathCtrlPoints(pathHandle,0,0,1,setContorlPoint(point) )
    end
    return pathHandle
end

local subs = function(x,y)
  local res ={}
  for i = 1,2 do
    res[i]= x[i]-y[i] 
  end 
  return res
end

local adds = function(x,y)
  local res ={}
  for i = 1,2 do
    res[i]= x[i]+y[i] 
  end 
  return res
end


function dump(o)
   if type(o) == 'table' then
      local s = '{ '
      for k,v in pairs(o) do
         if type(k) ~= 'number' then k = '"'..k..'"' end
         s = s .. '['..k..'] = ' .. dump(v) .. ','
      end
      return s .. '} '
   else
      return tostring(o)
   end
end


simGetPath = function(...)
  local file = ... or '/Users/redwannewaz/Projects/research@explore/path_tracking_vrep/track.csv'
  local csv = require 'include.csv'
  local s = {0,0.6}
  
  local data = csv.read(file,1,1)
  local X = data[1]
  -- transform line 
  local newdata = {}
  for i, k in pairs(data) do 
    newdata[i]=adds(subs(k,X),s)
  end
  return simGeneratePath(newdata)
end

