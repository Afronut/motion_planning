from math import cos, sin, radians, acos, degrees


class execise_7:
    def __init__(self, distances=[], angles=[], end_point=[]):
        self.distances = distances
        self.angles = angles
        self.end_point = end_point

    def compute(self):
        l1, l2, l3 = self.distances
        end_point_x = 0
        end_point_y = 0
        if not self.end_point:
            return self.get_coordonates(), self.angles
        elif not self.angles and self.end_point:
            end_point_x, end_point_y, theta = self.end_point
            self.angles = self.get_angles(end_point_x, end_point_y, theta)
            print("The configuration angles are: ", self.angles)
            ret = self.get_coordonates()
            return ret, self.angles

    def get_end_point(self):
        pass

    def get_angles(self, end_point_x, end_point_y, theta):
        l1, l2, l3 = self.distances
        theta1 = 0
        theta2 = 0
        theta3 = 0
        x3 = end_point_x - l3*cos(radians(theta))
        y3 = end_point_y - l3*sin(radians(theta))
        print(x3, y3)
        try:
            val = ((x3**2 + y3**2)-(l1**2+l2**2))/(l1*l2*2)
            theta2 = acos(val)
        except:
            print(" Theta 2 Solution not Found: Please reajust imput parameters\nValue cos(theta2): ",
                  val)
            exit()
        try:
            val = 1/(x3**3+y3**3)*(x3*(l1+l2*cos(theta2)) +
                                   y3*l2*(1 - cos(theta2)**2)**.5)
            theta1 = acos(val)
        except:
            try:
                val = 1/(x3**3+y3**3)*(x3*(l1+l2*cos(theta2)) +
                                       y3*l2*(1 - cos(theta2)**2)**.5)
                theta1 = acos(val)
            except:
                print("Theta3 Solution not Found: Please reajust imput parameters\nValue cos(theta3): ",
                      val)
                exit()

        theta3 = radians(theta)-(theta2 + theta1)

        return [round(degrees(theta1), 2), round(degrees(theta2), 2), round(degrees(theta3), 2)]

    def get_coordonates(self):
        x = 0
        y = 0
        theta = 0
        ret = []
        for i in range(0, len(self.distances)):
            theta += self.angles[i]
            ret.append([x, y])
            x = ret[i][0] + self.distances[i]*cos(radians(theta))
            y = ret[i][1] + self.distances[i]*sin(radians(theta))
        ret.append([x, y])
        return ret
