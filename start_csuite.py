import threading
import cmd, sys
from bullet.ControlSuite import ControlSuite
import numpy as np


class ControlSuiteShell(cmd.Cmd):
    intro = 'Welcome to the control suite shell.\nType help or ? to list commands.\n'
    prompt = '(dwl-csuite) '

    # Create the bullet simulator   
    csuite = ControlSuite()


    def do_step(self, arg):
        'Run a number of step simulations: step 10.\nThe default number is 1'
        nstep = 1
        try:
            nstep = int(arg)
        except ValueError:
            print "Not an integer, running only 1 step"

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
            print self.prompt, key

    def do_changeController(self, arg):
        'Change the controller: changeController controller_name.\nYou can see the list of controller by typing getControllerList'
        # Searching the controller
        plgs = self.csuite.controller_plugins
        for key in plgs:
            if key == arg:
                self.csuite.setController(plgs[key])
                return
        print 'The' + arg +' controller has not be found'

    def do_resetRobot(self, arg):
        'Reset the robot position'
        self.csuite.resetRobot()
    
    def do_fixedBase(self, arg):
        'Fix the base in the default robot position or defined one: fixedBase or fixedBase 0. 0. 1.'
        pos = []
        try:
            pos = tuple(map(float, arg.split()))
        except ValueError:
            print "Not an array of floats, fixed the base with the default posture"
        
        if not pos:
            self.csuite.fixedBase()
        elif len(pos) == 3:
            self.csuite.fixedBasePosition(np.asarray(pos))
        else:
            print 'You should pass a 3D position, using the default posture'
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
            print "Not a float, using the same time step"

if __name__ == '__main__':
    ControlSuiteShell().cmdloop()