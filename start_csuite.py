import threading
import cmd, sys
from bullet.ControlSuite import ControlSuite



class ControlSuiteShell(cmd.Cmd):
    intro = 'Welcome to the control suite shell.   Type help or ? to list commands.\n'
    prompt = '(sim) '

    # Create the bullet simulator   
    bullet_sim = ControlSuite()


    def do_step(self, arg):
        'Run a number of step simulations: step 10'
        ' The default number is 1'
        nstep = 1
        try:
            nstep = int(arg)
        except ValueError:
            print "Not an integer, running only 1 step"

        for i in range(nstep):
            self.bullet_sim.update()

    def do_run(self, arg):
        'Run the control suite in an infiny loop. You can stop it by typing stop'
        t = threading.Thread(target=self.bullet_sim.run, args=())
        t.daemon = True
        t.start()

    def do_stop(self, arg):
        'Stop the control suite loop. You can re-start it by typing run'
        self.bullet_sim.stop()

    def do_kill(self, arg):
        'Kill the control suite and its console'
        return True

    def do_getControllerList(self, arg):
        for key in self.bullet_sim.controller_plugins:
            print self.prompt, key

    def do_changeController(self, arg):
        # Searching the controller
        plgs = self.bullet_sim.controller_plugins
        for key in plgs:
            if key == arg:
                print plgs[key]
                self.bullet_sim.setController(plgs[key])
                print 'found controller'
                return

    def do_resetRobot(self, arg):
        'Reset the robot position'
        self.bullet_sim.resetRobotPosition()

    def do_enableROS(self, arg):
        'Enable the ROS communication'
        self.bullet_sim.enableROS()

    def do_print(self, arg):
        'Print the actual whole-body state'
        self.bullet_sim.printActualState()
    
    def do_setTimeStep(self, arg):
        'Set the time step of the simulation'
        p.setTimeStep(1.)

if __name__ == '__main__':
    ControlSuiteShell().cmdloop()