import dwl

class ws:
    actual = 0
    desired = 1

class Controller():
    def __init__(self, fbs, filename):
        self.fbs = fbs
        self.filename = filename
        self.yaml = dwl.YamlWrapper()
        self.yaml.setFile(self.filename)
        print 'Controller config file:', self.filename

    def start(self, states):
        return None
    
    def update(self, states):
        return None
    
    def stop(self):
        return None