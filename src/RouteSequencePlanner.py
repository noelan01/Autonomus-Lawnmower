#This is the program that controls the sequence in which the robot will move over the field
import math
#Fix so that the lengths used are the lengths from the front end website

class sequencePlanner:
    def __init__(self):
        
        #Initiating the pitch dimensions
        self.radius = 9.15
        self.sideLine = 120
        self.goalLine = 90
        self.goalAreaHeight = 5.5
        self.goalAreaWidth = 5.5+7.32+5.5
        self.penaltyAreaHeight = 16.5
        self.penaltyAreaWidth = 22+self.goalAreaWidth
        self.goalWidth = 7.32
        self.arcFromMid = math.sqrt(self.radius**2-5.5**2)
        self.lowerArcAngle = math.acos(5.5/self.radius)
        self.ppm = 100
        
    
    def outerLines(self,path):
        #Algorithm for painting the outer lines
        path.set_path(0,0,self.goalLine,0,self.ppm)
        path.set_path(self.goalLine,0,self.goalLine,self.sideLine,self.ppm)
        path.set_path(self.goalLine,self.sideLine,0,self.sideLine,self.ppm)
        path.set_path(0,self.sideLine,0,0,self.ppm)

    def lowerPenaltyArea(self,path):
        #Algorithm for painting the downer goal area of the field
        path.set_path(0,0,(self.goalLine-self.penaltyAreaWidth)/2,0,self.ppm)
        path.set_path((self.goalLine-self.penaltyAreaWidth)/2,0,(self.goalLine-self.penaltyAreaWidth)/2,self.penaltyAreaHeight,self.ppm)
        path.set_path((self.goalLine-self.penaltyAreaWidth)/2,self.penaltyAreaHeight,self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2,self.penaltyAreaHeight,self.ppm)
        path.set_path(self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2,self.penaltyAreaHeight,self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2,0,self.ppm)

    def lowerArc(self,path):
        path.set_path(self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2,0,self.goalLine/2-self.penaltyAreaWidth/2,0,self.ppm)
        path.set_path(self.goalLine/2-self.penaltyAreaWidth/2,0,self.goalLine/2-self.penaltyAreaWidth/2,self.penaltyAreaHeight,self.ppm)
        path.set_path(self.goalLine/2-self.penaltyAreaWidth/2,self.penaltyAreaHeight,self.goalLine/2-self.arcFromMid,self.penaltyAreaHeight,self.ppm)
        path.set_lower_arc_path(self.radius,(self.goalLine/2,11),2000)
        


    def lowerGoalArea(self,path):
        path.set_path(self.goalLine/2+self.arcFromMid,self.penaltyAreaHeight,self.goalLine/2+self.penaltyAreaWidth/2,self.penaltyAreaHeight,self.ppm)
        path.set_path(self.goalLine/2+self.penaltyAreaWidth/2,self.penaltyAreaHeight,self.goalLine/2+self.penaltyAreaWidth/2,0,self.ppm)
        path.set_path(self.goalLine/2+self.penaltyAreaWidth/2,0,self.goalLine/2+self.goalAreaWidth/2,0,self.ppm)
        path.set_path(self.goalLine/2+self.goalAreaWidth/2,0,self.goalLine/2+self.goalAreaWidth/2,self.goalAreaHeight,self.ppm)
        path.set_path(self.goalLine/2+self.goalAreaWidth/2,self.goalAreaHeight,self.goalLine/2-self.goalAreaWidth/2,self.goalAreaHeight,self.ppm)
        path.set_path(self.goalLine/2-self.goalAreaWidth/2,self.goalAreaHeight,self.goalLine/2-self.goalAreaWidth/2,0,self.ppm)

    def transportMidLine(self, path):
        path.set_path(self.goalLine/2-self.goalAreaWidth/2,0,0,self.sideLine/2,self.ppm)
    
    #Have to put quite a lot of points in the circle for it to follow
    def midLine(self,path):
        path.set_path(0,self.sideLine/2,self.goalLine/2-self.radius,self.sideLine/2,self.ppm)
        path.set_circle_path(self.radius,(self.goalLine/2,self.sideLine/2),3000)
        path.set_path(self.goalLine/2-self.radius,self.sideLine/2,self.goalLine,self.sideLine/2,self.ppm)

    def driveToUpperLineFromMid(self,path):
        path.set_path(self.goalLine,self.sideLine/2,self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2,self.sideLine,self.ppm)

    def upperPenaltyArea(self,path):
        path.set_path(self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2,self.sideLine,self.goalLine/2-self.penaltyAreaWidth/2,self.sideLine,self.ppm)
        path.set_path(self.goalLine/2-self.penaltyAreaWidth/2,self.sideLine,self.goalLine/2-self.penaltyAreaWidth/2,self.sideLine-self.penaltyAreaHeight,self.ppm)
        path.set_path(self.goalLine/2-self.penaltyAreaWidth/2,self.sideLine-self.penaltyAreaHeight,self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2,self.sideLine-self.penaltyAreaHeight,self.ppm)
        path.set_path(self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2,self.sideLine-self.penaltyAreaHeight,self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2,self.sideLine,self.ppm)

    def upperArc(self,path):
        path.set_path(self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2,self.sideLine,self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2,self.sideLine-self.penaltyAreaHeight,self.ppm)
        path.set_path(self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2,self.sideLine-self.penaltyAreaHeight,self.goalLine/2-self.arcFromMid,self.sideLine-self.penaltyAreaHeight,self.ppm)
        path.set_upper_arc_path(self.radius,(self.goalLine/2,self.sideLine-11),2000)
        path.set_path(self.goalLine/2+self.arcFromMid,self.sideLine-self.penaltyAreaHeight,self.goalLine/2+self.penaltyAreaWidth/2,self.sideLine-self.penaltyAreaHeight,self.ppm)
        path.set_path(self.goalLine/2+self.penaltyAreaWidth/2,self.sideLine-self.penaltyAreaHeight,self.goalLine/2+self.penaltyAreaWidth/2,self.sideLine,self.ppm)

    def upperGoalArea(self,path):
        path.set_path(self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2,self.sideLine,self.goalLine/2+self.goalAreaWidth/2,self.sideLine,self.ppm)
        path.set_path(self.goalLine/2+self.goalAreaWidth/2,self.sideLine,self.goalLine/2+self.goalAreaWidth/2,self.sideLine-self.goalAreaHeight,self.ppm)
        path.set_path(self.goalLine/2+self.goalAreaWidth/2,self.sideLine-self.goalAreaHeight,self.goalLine/2-self.goalAreaWidth/2,self.sideLine-self.goalAreaHeight,self.ppm)
        path.set_path(self.goalLine/2-self.goalAreaWidth/2,self.sideLine-self.goalAreaHeight,self.goalLine/2-self.goalAreaWidth/2,self.sideLine,self.ppm)

    def upperLeftCorner(self,path):
        path.set_path(self.goalLine/2-self.goalAreaWidth/2,self.sideLine,1,self.sideLine,self.ppm)
        path.set_upper_left_corner(1,(0,self.sideLine),100)
    def lowerLeftCorner(self,path):
        path.set_path(0,self.sideLine-1,0,1,self.ppm)
        path.set_bottom_left_corner(1,(0,0),100)
    
    def bottomRightCorner(self,path):
        path.set_path(1,0,self.goalLine-1,0,self.ppm)
        path.set_bottom_right_corner(1,(self.goalLine,0),100)
    
    def upperRightCorner(self,path):
        path.set_path(self.goalLine,1,self.goalLine,self.sideLine-1,self.ppm)
        path.set_upper_right_corner(1,(self.goalLine,self.sideLine),100)

        

    

