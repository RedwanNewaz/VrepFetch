from .core import vrepper

class SimulatorConnection(object):
    def __init__(self):
        self.venv = vrepper(port_num=19997, restart=True)
        self.venv.start()
    def start(self):
        self.venv.start_nonblocking_simulation()
    def stop(self):
        self.start_sync = False
        self.venv.stop_blocking_simulation()
        self.venv.end()