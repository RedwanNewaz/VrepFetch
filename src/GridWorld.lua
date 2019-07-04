local GridWorld ={}
local function map(x,y)
    local dx, dy,scale
    dx = math.abs(x[2]-x[1])
    dy = math.abs(y[2]-y[1])
    scale = dx/dy
    local r ={}
    for i = x[1],x[2],scale do 
        table.insert(r,i)
    end
    return r,scale
end

local function GetCoord(x_coord,y_coord,map_size, axis)
    local diffX = 1;
    local diffY = 1;
    if axis=='x' then
       diffX = map_size[1];
    else
       diffY = map_size[2];
    end

    local XY={};
    local nX = #x_coord;
    local nY = #y_coord;
    for x=1,nX,diffX do
        for y=1,nY,diffY do
            table.insert(XY,{x_coord[x],y_coord[y]})
        end
    end
    return XY
end 

local function drawVerticalLines(X)

    for i=1,#X,2 do
       GridWorld.drawEdge(X[i],X[i+1])
    end
end

local function drawHorizontalLines(X)
    local n = math.floor(#X/2);
    --print(#X,n)
    for i=1,n do
        --print(X[i],X[n+i])
       GridWorld.drawEdge(X[i],X[n+i])
    end
end


GridWorld.drawCells = function (box_val,map_size, edge_type)
    GridWorld.drawEdge = edge_type
    local x_coord,y_coord, dx, dy
    x_coord,dx = map({box_val[1],box_val[2]},{box_val[3],map_size[1]})
    y_coord,dy = map({box_val[1],box_val[4]},{box_val[3],map_size[2]})    
    local XY = GetCoord(x_coord,y_coord,map_size,'y');
    drawVerticalLines(XY)
    local XY = GetCoord(x_coord,y_coord,map_size,'x')
    --print('XY',XY)
    drawHorizontalLines(XY)
end
return GridWorld

