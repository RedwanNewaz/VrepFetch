PolarCoord = {
  __call = function(X)
    assert(#X>1,"table is not correct")
    local rho = math.sqrt(math.pow(X[1],2) + math.pow(X[2],2))
    local alpha = math.atan2(X[2],X[1])
    alpha = math.abs(alpha) * 180 / math.pi 
--    if(alpha<0) then 
--      alpha = math.pi - alpha 
--    end 
    local res = {rho,alpha}
    return res
  end,
  
  __index = function(X,k)
    print("calling index ", X)
  end,

  __add=function(X, Y) 
    assert(#X==#Y, "add op size does not match")
    local sum = {}
    X = X()
    Y = Y()
    for i = 1, #X do 
      sum[i] = X[i] + Y[i]
    end
    return sum  
  end,
  __sub=function(X,Y)
    assert(#X==#Y, "sub op size does not match")
    X = X()
    Y = Y()
    for i = 1, #X do 
      X[i] = X[i] - Y[i]
    end
  return X 
  
  end, 
  
  __eq = function(X,Y)
  
    return X.value == Y.value
  
  end

}