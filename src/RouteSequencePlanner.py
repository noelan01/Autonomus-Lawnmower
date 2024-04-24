import json

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
        self.ppm = 80

        self.file_path = "UserInputServer/data.json"

        self.load_pitch_data()

    def load_pitch_data(self):
        with open(self.file_path, 'r') as file:
            data = json.load(file)
        self.goalLine = round(float(data["shortside"]), 4)
        self.sideLine = round(float(data["longside"]), 4)
        
    
    def outerLines(self,path):
        #Algorithm for painting the outer lines
        path.set_path(0, 0, self.goalLine,0,self.ppm,"x")
        path.set_path(self.goalLine, 0, self.goalLine,self.sideLine, self.ppm,"y")
        path.set_path(self.goalLine, self.sideLine, 0, self.sideLine, self.ppm,"-x")
        path.set_path(0, self.sideLine, 0, 0, self.ppm,"-y")

    def lowerPenaltyArea(self,path):
        #Algorithm for painting the downer goal area of the field
        path.set_path(0, 0, (self.goalLine-self.penaltyAreaWidth)/2, 0, self.ppm,"x")
        path.set_path((self.goalLine-self.penaltyAreaWidth)/2, 0, (self.goalLine-self.penaltyAreaWidth)/2, self.penaltyAreaHeight, self.ppm,"y")
        path.set_path((self.goalLine-self.penaltyAreaWidth)/2, self.penaltyAreaHeight, self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2, self.penaltyAreaHeight, self.ppm,"x")
        path.set_path(self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2, self.penaltyAreaHeight, self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2, 0, self.ppm,"-y")

    def lowerArc(self,path):
        path.set_path(self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2, 0, self.goalLine/2-self.penaltyAreaWidth/2, 0, self.ppm,"-x")
        path.set_path(self.goalLine/2-self.penaltyAreaWidth/2, 0, self.goalLine/2-self.penaltyAreaWidth/2, self.penaltyAreaHeight, self.ppm,"y")
        path.set_path(self.goalLine/2-self.penaltyAreaWidth/2, self.penaltyAreaHeight, self.goalLine/2-self.arcFromMid, self.penaltyAreaHeight, self.ppm,"x")
        path.set_lower_arc_path(self.radius, (self.goalLine/2, 11), 8000,"None")
        


    def lowerGoalArea(self,path):
        path.set_path(self.goalLine/2+self.arcFromMid, self.penaltyAreaHeight, self.goalLine/2+self.penaltyAreaWidth/2, self.penaltyAreaHeight, self.ppm,"x")
        path.set_path(self.goalLine/2+self.penaltyAreaWidth/2, self.penaltyAreaHeight, self.goalLine/2+self.penaltyAreaWidth/2, 0, self.ppm,"-y")
        path.set_path(self.goalLine/2+self.penaltyAreaWidth/2, 0, self.goalLine/2+self.goalAreaWidth/2, 0, self.ppm,"-x")
        path.set_path(self.goalLine/2+self.goalAreaWidth/2,0, self.goalLine/2+self.goalAreaWidth/2, self.goalAreaHeight, self.ppm,"y")
        path.set_path(self.goalLine/2+self.goalAreaWidth/2, self.goalAreaHeight, self.goalLine/2-self.goalAreaWidth/2, self.goalAreaHeight, self.ppm,"-x")
        path.set_path(self.goalLine/2-self.goalAreaWidth/2, self.goalAreaHeight, self.goalLine/2-self.goalAreaWidth/2, 0, self.ppm,"-y")

    def transportMidLine(self, path):
        path.set_path(self.goalLine/2-self.goalAreaWidth/2, 0, 0, self.sideLine/2, self.ppm,"None")
    
    #Have to put quite a lot of points in the circle for it to follow
    def midLine(self,path):
        path.set_path(0, self.sideLine/2, self.goalLine/2-self.radius, self.sideLine/2, self.ppm,"x")
        path.set_circle_path(self.radius, (self.goalLine/2,self.sideLine/2), 8000,"None")
        path.set_path(self.goalLine/2-self.radius, self.sideLine/2, self.goalLine,self.sideLine/2, self.ppm,"x")

    def driveToUpperLineFromMid(self,path):
        path.set_path(self.goalLine,self.sideLine/2, self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2, self.sideLine,self.ppm,"None")

    def upperPenaltyArea(self,path):
        path.set_path(self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2, self.sideLine,self.goalLine/2-self.penaltyAreaWidth/2, self.sideLine,self.ppm,"-x")
        path.set_path(self.goalLine/2-self.penaltyAreaWidth/2,self.sideLine, self.goalLine/2-self.penaltyAreaWidth/2, self.sideLine-self.penaltyAreaHeight, self.ppm,"-y")
        path.set_path(self.goalLine/2-self.penaltyAreaWidth/2,self.sideLine-self.penaltyAreaHeight, self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2, self.sideLine-self.penaltyAreaHeight, self.ppm,"x")
        path.set_path(self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2, self.sideLine-self.penaltyAreaHeight, self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2, self.sideLine,self.ppm,"y")

    def upperArc(self,path):
        path.set_path(self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2, self.sideLine,self.goalLine/2-self.penaltyAreaWidth/2,self.sideLine,self.ppm,"-x")
        path.set_path(self.goalLine/2-self.penaltyAreaWidth/2,self.sideLine,self.goalLine/2-self.penaltyAreaWidth/2,
                        self.sideLine-self.penaltyAreaHeight, self.ppm,"-y")
        path.set_path(self.goalLine/2-self.penaltyAreaWidth/2,
                        self.sideLine-self.penaltyAreaHeight, self.goalLine/2-self.arcFromMid, 
                        self.sideLine-self.penaltyAreaHeight,self.ppm,"x")
        path.set_upper_arc_path(self.radius, (self.goalLine/2, self.sideLine-11),8000,"None")
        path.set_path(self.goalLine/2+self.arcFromMid, self.sideLine-self.penaltyAreaHeight, self.goalLine/2+self.penaltyAreaWidth/2, self.sideLine-self.penaltyAreaHeight, self.ppm,"x")
        path.set_path(self.goalLine/2+self.penaltyAreaWidth/2, self.sideLine-self.penaltyAreaHeight, self.goalLine/2+self.penaltyAreaWidth/2, self.sideLine, self.ppm,"y")

    def upperGoalArea(self,path):
        path.set_path(self.goalLine-(self.goalLine-self.penaltyAreaWidth)/2, self.sideLine, self.goalLine/2+self.goalAreaWidth/2, self.sideLine, self.ppm,"-x")
        path.set_path(self.goalLine/2+self.goalAreaWidth/2, self.sideLine,self.goalLine/2+self.goalAreaWidth/2, self.sideLine-self.goalAreaHeight, self.ppm,"-y")
        path.set_path(self.goalLine/2+self.goalAreaWidth/2, self.sideLine-self.goalAreaHeight, self.goalLine/2-self.goalAreaWidth/2, self.sideLine-self.goalAreaHeight, self.ppm,"x")
        path.set_path(self.goalLine/2-self.goalAreaWidth/2, self.sideLine-self.goalAreaHeight, self.goalLine/2-self.goalAreaWidth/2, self.sideLine, self.ppm,"y")

    def upperLeftCorner(self,path):
        path.set_path(self.goalLine/2-self.goalAreaWidth/2,self.sideLine,1,self.sideLine,self.ppm,"-x")
        path.set_upper_left_corner(1,(0,self.sideLine),400,"None")
    def lowerLeftCorner(self,path):
        path.set_path(0,self.sideLine-1,0,1,self.ppm,"-y")
        path.set_bottom_left_corner(1,(0,0),400,"None")
    
    def bottomRightCorner(self,path):
        path.set_path(1,0,self.goalLine-1,0,self.ppm,"x")
        path.set_bottom_right_corner(1,(self.goalLine,0),400,"None")
    
    def upperRightCorner(self,path):
        path.set_path(self.goalLine,1,self.goalLine,self.sideLine-1,self.ppm,"y")
        path.set_upper_right_corner(1,(self.goalLine,self.sideLine),400,"None")

        

    

