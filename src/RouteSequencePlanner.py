#This is the program that controls the sequence in which the robot will move over the field

class sequencePlanner():
    def __init__(self):
        
        #Initiating the pitch dimensions
        self.radius = 9.15
        self.sideLine = 120
        self.goalLine = 90
        self.goalAreaHeight = 5.5
        self.goalAreaWidth = 5.5+7.32+5.5
        self.penaltyAreaHeight = 16.5
        self.penaltyAreaWidth = 22+self.goalAreaWidth


    def print(self):
        goalLine = self.goalLine
        print(self.goalLine)

    




