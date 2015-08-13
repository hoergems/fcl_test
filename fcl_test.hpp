#include <iostream>
#include "fcl/BVH/BVH_model.h"
#include "fcl/BV/BV.h"
#include "Kinematics.hpp"
#include "Obstacle.hpp"
#include "fcl/math/matrix_3f.h"

namespace fcl_test {

class FclTest {
    public: 
        FclTest();
        
        void funct();
        
        std::vector<fcl::OBB> createManipulatorCollisionStructures(std::vector<double> &joint_angles);
        
    private:
        fcl::AABB link_aabb;
        
        shared::Kinematics kinematics_;       
    
};

}
