require 'class'
pid = class(function(instance,name)
    instance._name = name
    instance._dt=sim.getSimulationTimeStep()
    instance._max=1
    instance._min=0
    instance._Kp=0
    instance._Kd=0
    instance._Ki=0
    instance._pre_error=0
    instance._integral=0
 end)

function pid:set(kp,kd,ki)
    self._Kp = kp
    self._Kd = kd
    self._Ki = ki
end

function pid:print()
    print(self._name," pid controller!")
end

function pid:calculate(setpoint, pv)
    err = setpoint - pv
    -- print(self._name,err)
    pout = self._Kp * err
    self._integral = self._integral + err *self._dt
    iout = self._integral * self._Ki
    derivative = (err - self._pre_error)/self._dt
    dout = self._Kd * derivative
    out = pout + iout + dout
    if (out>self._max) then
        out = self._max
    elseif (out<self._min) then
        out = self._min
    end
    self._pre_error = err
    return out
end