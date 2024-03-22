#This is the program that controls the sequence in which the robot will move over the field

#Fix so that the lengths used are the lengths from the front end website

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
        
    
    def outerLines(self,path):
        path.set_path(0,0,self.goalLine,0,100)
        path.set_path(self.goalLine,0,self.goalLine,self.sideLine,100)
        path.set_path(self.goalLine,self.sideLine,0,self.sideLine,100)
        path.set_path(0,self.sideLine,0,0,100)

    def goalArea(self,path):
        path.set_path(0,0,(self.goalLine-self.penaltyAreaWidth)/2,0,100)
        path.set_path((self.goalLine-self.penaltyAreaWidth)/2,0,(self.goalLine-self.penaltyAreaWidth)/2,self.penaltyAreaHeight,100)
        path.set_path((self.goalLine-self.penaltyAreaWidth)/2,self.penaltyAreaHeight,self.goalLine-((self.goalLine-self.penaltyAreaWidth)/2),self.penaltyAreaHeight,100)
        path.set_path(self.goalLine-((self.goalLine-self.penaltyAreaWidth)/2),self.penaltyAreaHeight,self.goalLine-((self.goalLine-self.penaltyAreaWidth)/2),0,100)

    def midLine()
        
    
        
        

        



    




