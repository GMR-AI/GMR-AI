import sim

class RobotControl:
    '''
    Simple robot interface for ease of use
    '''
    def __init__(self, port, left_wheels, right_wheels):
        self.clientID = connect(port)
        self.left_wheels = [sim.simxGetObjectHandle(self.clientID, left_wheels[0], sim.simx_opmode_blocking)[1],
                            sim.simxGetObjectHandle(self.clientID, left_wheels[1], sim.simx_opmode_blocking)[1],]
        self.right_wheels = [sim.simxGetObjectHandle(self.clientID, right_wheels[0], sim.simx_opmode_blocking)[1],
                            sim.simxGetObjectHandle(self.clientID, right_wheels[1], sim.simx_opmode_blocking)[1],]
    
    def stop(self):
        '''
        Stops the robot movement
        '''
        # m/s to rad/s conversion
        for wheel in self.left_wheels:
            sim.simxSetJointTargetVelocity(self.clientID, wheel, 0, sim.simx_opmode_blocking)

        for wheel in self.right_wheels:
            sim.simxSetJointTargetVelocity(self.clientID, wheel, 0, sim.simx_opmode_blocking)

    def forward(self, speed: float | int):
        '''
        Makes the robot go forward at the desired speed in m/s
        '''
        # m/s to rad/s conversion
        wheels_speed = speed
        sim.simxSetJointTargetVelocity(self.clientID, self.left_wheels[0], wheels_speed, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(self.clientID, self.right_wheels[0], -wheels_speed, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(self.clientID, self.left_wheels[1], wheels_speed, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(self.clientID, self.right_wheels[1], -wheels_speed, sim.simx_opmode_blocking)
    
    def backward(self, speed: float | int):
        '''
        Makes the robot go forward at the desired speed in m/s
        '''
        # m/s to rad/s conversion
        wheels_speed = speed
        sim.simxSetJointTargetVelocity(self.clientID, self.left_wheels[0], -wheels_speed, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(self.clientID, self.right_wheels[0], wheels_speed, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(self.clientID, self.left_wheels[1], -wheels_speed, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(self.clientID, self.right_wheels[1], wheels_speed, sim.simx_opmode_blocking)

def connect(port):
# returns the client number or -1 if it cannot connect
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',port,True,True,2000,5) # Connect
    if clientID >= 0: print("conectado a", port)
    else: print("no se pudo conectar")
    return clientID

if __name__ == '__main__':
    left_wheels = ['./back_left_wheel', './front_left_wheel'] # Left Wheels Names
    right_wheels = ['./back_right_wheel', './front_right_wheel'] # Right Wheels Names
    r = RobotControl(19999, left_wheels, right_wheels)
    r.forward(1)