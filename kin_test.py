from kin import *
from util import *
from obstacle import *

class Test:
    def __init__(self):
        self.ultis = Utils()
        self.kinematics = self.set_kinematics()
        obstacle = self.create_obstacle()
        print obstacle.inCollision(self.get_manipulator_collision_structure())
       
    def create_obstacle(self):
        terrain = Terrain("terr", 0.0, 1.0, False)
        obstacle = Obstacle(2.5, 1.5, 0.0, 1.0, 1.0, 1.0, terrain)
        return obstacle
        
    def get_manipulator_collision_structure(self):
        joint_angles = v_double()
        joint_angles[:] = [0.5, 0.0, 0.0]

        u = Utils()
        collision_structures = u.createManipulatorCollisionStructures(joint_angles, self.kinematics)
        return collision_structures
        
    def set_kinematics(self):
        links = v2_double()
        axis = v2_int()
        
        link = v_double()
        ax1 = v_int()
        ax2 = v_int()
        
        link[:] = [1.0, 0.0, 0.0]
        links[:] = [link for i in xrange(3)]
        
        ax1[:] = [0, 0, 1]
        ax2[:] = [0, 1, 0]
        
        axis[:] = [ax1, ax2, ax1]
        
        k = Kinematics()
        k.setLinksAndAxis(links, axis)
        
        return k
        

if __name__ == "__main__":
    Test()