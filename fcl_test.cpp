#include "fcl_test.hpp"

using std::cout;
using std::endl;

using namespace fcl;
using namespace shared;


namespace fcl_test {

FclTest::FclTest() :
    link_aabb(Vec3f(0.0, -0.0025, -0.0025), Vec3f(1.0, 0.0025, 0.0025)),
    kinematics_({{1.0, 0.0, 0.0},
                 {1.0, 0.0, 0.0},
                 {1.0, 0.0, 0.0}},
                {{0, 0, 1},
                 {0, 1, 0},
                 {0, 0, 1}}) {    
}

std::vector<OBB> FclTest::createManipulatorCollisionStructures(std::vector<double> &joint_angles) {
    std::vector<OBB> collision_structures;
    int n = 0;
    for (size_t i = 0; i < joint_angles.size(); i++) {
        const std::pair<Vec3f, Matrix3f> pose_link_n = kinematics_.getPoseOfLinkN(joint_angles, n);
        OBB obb;
        convertBV(link_aabb, Transform3f(pose_link_n.second, pose_link_n.first), obb);
        collision_structures.push_back(obb);
        n++;
    }
    
    return collision_structures;
}

void FclTest::funct() {    
    const std::vector<double> joint_angles({0.0666667, -0.0666667, -0.0666667});
    for (int n = 0; n < 4; n++) {
        const std::pair<Vec3f, Matrix3f> pose = kinematics_.getPoseOfLinkN(joint_angles, n);
        std::vector<double> position = kinematics_.getPositionOfLinkN(joint_angles, n);    
        cout << "pose: " << pose.first << endl;
        cout << "position: " << position[0] << ", " << position[1] << ", " << position[2] << endl;
    }
    
    std::vector<double> ee_position = kinematics_.getEndEffectorPosition(joint_angles);
    const std::pair<Vec3f, Matrix3f> ee_pose = kinematics_.getEndEffectorPose(joint_angles); 
    cout << "ee position " << ee_position[0] << ", " << ee_position[1] << ", " << ee_position[2]  << endl;
    cout << "ee pose " << ee_pose.first << ", " << ee_pose.second << endl;
    
   
    /**BVHModel<OBB> m1;
    BVHModel<OBB> m2;
    Matrix3f rot(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    
    // Generate transform for first link
    int n = 1;
    const std::pair<Vec3f, Matrix3f> pose1 = kinematics_.getPoseOfLinkN(joint_angles, n);
    Transform3f rotate_transform1(pose1.second, pose1.first);    
    
    // Create collision structure for the first link
    Vec3f p1_vec(0.0, -0.025, -0.025);
    Vec3f p2_vec(1.0, 0.025, 0.025);    
    AABB collision_structure1(p1_vec, p2_vec);
    
    OBB obb1;
    convertBV(collision_structure1, rotate_transform1, obb1);   
    
    cout << "center obb: " << obb1.center() << endl; 
    
    // Create collition structure for the first obstacle
    n = 0;
    const std::pair<Vec3f, Matrix3f> pose2 = kinematics_.getPoseOfLinkN(joint_angles, n);
    Transform3f rotate_transform2(pose2.second, pose2.first); 
    Vec3f obst_p1(-0.5, -0.5, 0.0);
    Vec3f obst_p2(0.5, 0.5, 1.0);
    AABB collision_structure2(obst_p1, obst_p2);
    
    OBB obb2;
    convertBV(collision_structure2, rotate_transform2, obb2);
    
    if (obb1.overlap(obb2)) {
        cout << "OVERLAP!!" << endl;
    }
    else {
        cout << "No overlap" << endl;
    }*/
}

}

int main(int argc, char** argv) {   
   fcl_test::FclTest fcl;
   Terrain t1("t1", 1.0, 1.0, true);
   Obstacle obst1(3.0, 0.51, 0.0, 1.0, 1.0, 1.0, t1);
   
   std::vector<double> joint_angles({0.0, 0.5, 0.0});
   std::vector<OBB> manipulator_collision_structures = fcl.createManipulatorCollisionStructures(joint_angles);
   
   bool collides = obst1.in_collision(manipulator_collision_structures);
   if (collides) {
       cout << "COLLIDES!!!" << endl;
   }
   else {
       cout << "NO COLLISION" << endl;
   }
   fcl.funct();
   
   std::vector<double> p({3.0, 0.0, 0.0});
   bool contain = obst1.in_collision(p);
   if (contain) {
      cout << "CONTAINS" << endl;
   }
   else{
      cout << "CONTAINS NOT" << endl;
   }
   
   return 0;
}
