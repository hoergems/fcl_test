#include "fcl_test.hpp"

using std::cout;
using std::endl;

using namespace fcl;
using namespace shared;


namespace fcl_test {

FclTest::FclTest() {    
}

void FclTest::funct() {
    Kinematics kinematics;
    const std::vector<double> joint_angles({0.0, -M_PI / 4.0, M_PI / 2.0});
    for (int n = 0; n < 4; n++) {
        const std::pair<Vec3f, Matrix3f> pose = kinematics.getPoseOfLinkN(joint_angles, n);    
        cout << pose.first << endl;
    }
    BVHModel<OBBRSS> m1;
    BVHModel<OBBRSS> m2;
    Matrix3f rot(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    
}

}

int main(int argc, char** argv) {   
   fcl_test::FclTest fcl;
   fcl.funct();
   return 0;
}
