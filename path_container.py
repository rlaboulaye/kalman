from threading import Lock

class PathContainer(object):


    def __init__(self, path=None):
        self.lock = Lock()
        self.path = path
        self.changed = False


    def get_path(self, reset_changed=True):
        self.lock.acquire()
        path = self.path
        if reset_changed:
            self.changed = False
        self.lock.release()
        return path


    def set_path(self, path):
        self.lock.acquire()
        self.path = path
        self.changed = True
        self.lock.release()


    def is_changed(self):
        return self.changed
