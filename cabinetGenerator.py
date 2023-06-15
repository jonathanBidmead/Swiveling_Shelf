#creating a cabinet object which has a thickness, and which can be specified with the interior dimensions (list of points).
#inside corners must be specified clockwise from the left edge of the door.
#the door space is automatically calculated as the segment between the last two specified points
class cabinet:
    def __init__(self,insideCorners,thickness):
        self.insideCorners=insideCorners
        self.thickness=thickness

    


cab1 = cabinet([1,1],2)
print(cab1.test)