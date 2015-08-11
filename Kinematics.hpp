#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_

#include <iostream>
#include <vector>
#include <algorithm>
#include "fcl/math/matrix_3f.h"
#include <Eigen/Dense>


using namespace fcl;

namespace shared {

    class Kinematics {
        public:
            Kinematics() = default;
                         
            /* Gets the end effector position for a given set of joint angles */                                
            std::vector<double> getEndEffectorPosition(const std::vector<double> &joint_angles) const;
            
            std::vector<double> getPositionOfLinkN(const std::vector<double> &joint_angles, int &n) const;
            
            std::pair<Vec3f, Matrix3f> getPoseOfLinkN(const std::vector<double> &joint_angles, int &n) const;
            
            
            
        private:
        
            /* Gets an element of SE2 for a rotation (theta) and translation (x, y) */
            std::vector<std::vector<double>> getSE2(const double &theta, double x, double y) const;
            
            /* Multiply two elements of SE2 */
            std::vector<std::vector<double>> multiplySE2(const std::vector<std::vector<double>> &SE21, 
                                                         const std::vector<std::vector<double>> &SE22) const;
            
            Eigen::MatrixXd getTransformationMatr(double sigma_n, double d_n, double a_n, double alpha_n) const;
            
            
            
    };



}

#endif
