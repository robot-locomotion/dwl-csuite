import threading
import cmd, sys, os
from bullet.ControlSuite import ControlSuite, bcolors
import numpy as np


class ControlSuiteShell(cmd.Cmd):
    intro = 'Welcome to the dwl control suite shell.\nType help or ? to list commands.\n'
    prompt = '(dwl-csuite) '

    def __init__(self):
        cmd.Cmd.__init__(self)

        # Create the bullet simulator
        root_path = os.path.dirname(os.path.abspath(__file__)) + '/config/'
        filename = root_path + 'dwl_csuite.yaml'
        nargs = len(sys.argv)
        if nargs == 1:
            print bcolors.OKBLUE + 'Openning the default dwl_csuite config file:', \
                  filename + bcolors.ENDC
        elif nargs == 2:
            filename = root_path + '/' + sys.argv[1]
            print bcolors.OKBLUE + 'Openning the', filename, 'config file' + bcolors.ENDC
        elif nargs == 3:
            root_path = sys.argv[1]
            filename = root_path + '/' + sys.argv[2]
            print bcolors.OKBLUE + 'Openning the', filename, 'config file' + bcolors.ENDC
        else:
            print bcolors.FAIL + 'Wrong input, running the default dwl_csuite config file' \
                 + bcolors.ENDC
        self.csuite = ControlSuite(filename)

    def do_step(self, arg):
        'Run a number of step simulations: step 10.\nThe default number is 1'
        nstep = 1
        try:
            nstep = int(arg)
        except ValueError:
            print bcolors.WARNING + "Not an integer, running only 1 step" + bcolors.ENDC

        for i in range(nstep):
            self.csuite.update()

    def do_run(self, arg):
        'Run the control suite in an infiny loop.\nYou can stop it by typing stop'
        t = threading.Thread(target=self.csuite.run, args=())
        t.daemon = True
        t.start()

    def do_stop(self, arg):
        'Stop the control suite loop.\nYou can re-start it by typing run'
        self.csuite.stop()

    def do_quit(self, arg):
        'Quit the control suite and its console'
        return True

    def do_getControllerList(self, arg):
        'Get a list of controller.\nThe controller are Python modules with csuite as prefix name'
        for key in self.csuite.controller_plugins:
            print key

    def do_changeController(self, arg):
        'Change the controller: changeController controller_name config_file (optional).\nYou can see the list of controller by typing getControllerList'
        # Searching the controller
        ctrl_info = tuple(map(str, arg.split()))
        nin = len(ctrl_info)
        if nin == 0:
            print 'You need to pass the controller name and config file (optional)'
        elif nin == 1:
            self.csuite.changeController(ctrl_info[0], '')
        elif nin == 2:
            self.csuite.changeController(ctrl_info[0], ctrl_info[1])
        else:
            print 'Wrong inputs'

    def do_resetRobot(self, arg):
        'Reset the robot position'
        self.csuite.resetRobot()
    
    def do_fixedBase(self, arg):
        'Fix the base in the default robot position or defined one: fixedBase or fixedBase 0. 0. 1.'
        pos = []
        try:
            pos = tuple(map(float, arg.split()))
        except ValueError:
            print bcolors.WARNING + 'Not an array of floats, fixed the base with the default posture' \
                  + bcolors.ENDC
        
        if not pos:
            self.csuite.fixedBase()
        elif len(pos) == 3:
            self.csuite.fixedBasePosition(np.asarray(pos))
        else:
            print bcolors.WARNING + 'You should pass a 3D position, using the default posture' \
                  + bcolors.ENDC
            self.csuite.fixedBase()

    def do_enableROS(self, arg):
        'Enable the ROS communication'
        self.csuite.enableROS()

    def do_printActualState(self, arg):
        'Print the actual whole-body state'
        self.csuite.printActualState()
    
    def do_printDesiredState(self, arg):
        'Print the desired whole-body state'
        self.csuite.printDesiredState()

    def do_setTimeStep(self, arg):
        'Set the time step of the simulation'
        try:
            time_step = float(arg)
            self.csuite.biface.setTimeStep(time_step)
            self.csuite.time_step = time_step
        except ValueError:
            print bcolors.WARNING + "Not a float, using the same time step" + bcolors.ENDC

if __name__ == '__main__':
    ControlSuiteShell().cmdloop()