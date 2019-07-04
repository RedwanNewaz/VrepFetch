from .lib.vrepConst import sim_jointfloatparam_velocity, simx_opmode_buffer, simx_opmode_streaming, simx_opmode_oneshot
from .utils import check_ret, blocking, oneshot
import numpy as np
from time import sleep

class vrepobject:
    def __init__(self, env, handle, is_joint=True):
        self.env = env
        self.handle = handle
        self.is_joint = is_joint

    def get_orientation(self, relative_to=None):
        eulerAngles, = check_ret(self.env.simxGetObjectOrientation(
            self.handle,
            -1 if relative_to is None else relative_to.handle,
            oneshot))
        return eulerAngles

    def get_position(self, relative_to=None):
        position, = check_ret(self.env.simxGetObjectPosition(
            self.handle,
            -1 if relative_to is None else relative_to.handle,
            oneshot))
        return position

    def get_velocity(self):
        return check_ret(self.env.simxGetObjectVelocity(
            self.handle,
            # -1 if relative_to is None else relative_to.handle,
            oneshot))
        # linearVel, angularVel

    def set_velocity(self, v):
        self._check_joint()
        return check_ret(self.env.simxSetJointTargetVelocity(
            self.handle,
            v,
            simx_opmode_oneshot))

    def set_force(self, f):
        self._check_joint()
        return check_ret(self.env.simxSetJointForce(
            self.handle,
            f,
            oneshot))

    def set_position_target(self, angle):
        """
        Set desired position of a servo

        :param int angle: target servo angle in degrees
        :return: None if successful, otherwise raises exception
        """
        self._check_joint()
        return check_ret(self.env.simxSetJointTargetPosition(
            self.handle,
            -np.deg2rad(angle),
            oneshot))



    def force_position(self, angle):
        """
        Force desired position of a servo

        :param int angle: target servo angle in degrees
        :return: None if successful, otherwise raises exception
        """
        self._check_joint()
        return check_ret(self.env.simxSetJointPosition(
            self.handle,
            -np.deg2rad(angle),
            oneshot))

    def set_position(self, x, y, z):
        """
        Set object to specific position (should never be done with joints)
        :param pos:  tuple or list with 3 coordinates
        :return: None
        """
        pos = (x, y, z)
        return check_ret(self.env.simxSetObjectPosition(self.handle, -1, pos, oneshot))

    def set_orientation(self, ori):
        """
        Set object to specific position (should never be done with joints)
        :param pos:  tuple or list with 3 coordinates
        :return: None
        """

        return check_ret(self.env.simxSetObjectOrientation(self.handle, -1, ori, oneshot))

    def get_joint_angle(self):
        self._check_joint()
        angle = check_ret(
            self.env.simxGetJointPosition(
                self.handle,
                oneshot
            )
        )
        return -np.rad2deg(angle[0])

    def get_joint_force(self):
        self._check_joint()
        force = check_ret(
            self.env.simxGetJointForce(
                self.handle,
                oneshot
            )
        )
        return force

    def get_joint_velocity(self):
        self._check_joint()
        vel = check_ret(self.env.simxGetObjectFloatParameter(
            self.handle,
            sim_jointfloatparam_velocity,
            oneshot
        ))
        return vel

    def read_force_sensor(self):
        state, forceVector, torqueVector = check_ret(self.env.simxReadForceSensor(
            self.handle,
            oneshot))

        if state & 1 == 1:
            return None  # sensor data not ready
        else:
            return forceVector, torqueVector

    def get_vision_image(self):
        res, nim = False, None

        try:
            resolution, image = check_ret(self.env.simxGetVisionSensorImage(
                self.handle,
                0,  # options=0 -> RGB
                simx_opmode_streaming,
            ), ignore_one=True)
            dim, im = resolution, image
            nim = np.array(im, dtype='uint8')
            nim = np.reshape(nim, (dim[1], dim[0], 3))
            nim = np.flip(nim, 0)  # LR flip
            nim = np.flip(nim, 2)  # RGB -> BGR
            res = True
        except:
            print('image cannot retrive')

        return res, nim

    def _check_joint(self):
        if not self.is_joint:
            raise Exception("Trying to call a joint function on a non-joint object.")

    def get_global_variable(self, name, is_first_time):
        if is_first_time:
            return self.env.simxGetFloatSignal(self.cid, name, simx_opmode_streaming)
        else:
            return self.env.simxGetFloatSignal(self.cid, name, simx_opmode_buffer)

    def set_string_signal(self,u_t,name):
        # self._check_joint()
        return check_ret(self.env.simxSetStringSignal(
            name,
            self.env.simxPackFloats(u_t),
            oneshot))
