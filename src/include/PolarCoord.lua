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

randomGoal = function()
    local random_axis = function()
      local x = math.random()*10-5
      return math.fmod(x,4)
    end
    local x =  random_axis()
    local y =  random_axis()
    return {x,y,0.05}
end

--print(dump(randomGoal()))
for i = 1,3 do
pose = randomGoal()
print(dump(pose))

end

