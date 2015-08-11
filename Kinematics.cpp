#include "Kinematics.hpp"

using std::cout;
using std::endl;


namespace shared {

/* Gets the end effector position for a given set of joint angles */    
std::vector<double> Kinematics::getEndEffectorPosition(const std::vector<double> &joint_angles) const {
    std::vector<std::vector<double>> matr = multiplySE2(multiplySE2(getSE2(joint_angles[0], 1.0, 0.0), 
                                                                    getSE2(joint_angles[1], 1.0, 0.0)), 
                                                        getSE2(joint_angles[2], 1.0, 0.0));    
    std::vector<double> ee_position;
    ee_position.push_back(matr[0][2]);
    ee_position.push_back(matr[1][2]);
    return ee_position;
}  

std::vector<double> Kinematics::getPositionOfLinkN(const std::vector<double> &joint_angles, int &n) const {
    std::vector<double> pos;
    std::vector<std::vector<double>> matr = getSE2(joint_angles[0], 1.0, 0.0);    
    if (n == 0) {
        pos.push_back(matr[0][2]);
        pos.push_back(matr[1][2]);
        return pos;
    }
    
    for (int i=1; i < n + 1; i++) {
        matr = multiplySE2(matr, getSE2(joint_angles[i], 1.0, 0.0));        
    }
    pos.push_back(matr[0][2]);
    pos.push_back(matr[1][2]); 
    return pos;
}

/* Gets an element of SE2 for a rotation (theta) and translation (x, y) */
std::vector<std::vector<double>> Kinematics::getSE2(const double &theta, double x, double y) const{
    std::vector< std::vector<double> > matrix(3, std::vector<double>(3));
    matrix[0][0] = cos(theta);
    matrix[1][0] = sin(theta);
    matrix[2][0] = 0.0;
    matrix[0][1] = -sin(theta);
    matrix[1][1] = cos(theta);
    matrix[2][1] = 0.0;
    matrix[0][2] = x * cos(theta);
    matrix[1][2] = x * sin(theta) + y * cos(theta);
    matrix[2][2] = 1.0;
    
    return matrix;

} 

std::pair<Vec3f, Matrix3f> Kinematics::getPoseOfLinkN(const std::vector<double> &joint_angles, int &n) const {
   Eigen::MatrixXd res(4, 4);
   if (n == 0) {
       res = getTransformationMatr(joint_angles[0], 0.0, 0.0, 0.0);
       cout << res << endl;       
   }
   else if (n == 1) {
       Eigen::MatrixXd a = getTransformationMatr(joint_angles[0], 0.0, 1.0, -M_PI / 2.0);
       Eigen::MatrixXd b = getTransformationMatr(joint_angles[1], 0.0, 0.0, 0.0);
       res = a*b;
       cout << res << endl;
       
   }
   else if (n == 2) {       
       Eigen::MatrixXd a = getTransformationMatr(joint_angles[0], 0.0, 1.0, -M_PI / 2.0);
       Eigen::MatrixXd b = getTransformationMatr(joint_angles[1], 0.0, 1.0, 0.0);
       Eigen::MatrixXd c = getTransformationMatr(joint_angles[2], 0.0, 0.0, 0.0);       
       res = a*(b*c);
       cout << res << endl;
   }
   else if (n == 3) {       
       Eigen::MatrixXd a = getTransformationMatr(joint_angles[0], 0.0, 1.0, -M_PI / 2.0);
       Eigen::MatrixXd b = getTransformationMatr(joint_angles[1], 0.0, 1.0, 0.0);
       Eigen::MatrixXd c = getTransformationMatr(joint_angles[2], 0.0, 1.0, 0.0);      
       res = a*(b*c);
       cout << res << endl;       
   } 
   
   Vec3f r_vec = Vec3f(res(0, 3), res(1, 3), res(2, 3));
   Matrix3f r_matr = Matrix3f(res(0, 0), res(0, 1), res(0, 2), 
                              res(1, 0), res(1, 1), res(1, 2), 
                              res(2, 0), res(2, 1), res(2, 2));
   auto p = std::make_pair(r_vec, r_matr);
   return p;
}

Eigen::MatrixXd Kinematics::getTransformationMatr(double sigma_n, double d_n, double a_n, double alpha_n) const {
    Eigen::MatrixXd b(4,4);    
    b << cos(sigma_n), -sin(sigma_n) * cos(alpha_n), sin(sigma_n) * sin(alpha_n), a_n * cos(sigma_n),
         sin(sigma_n), cos(sigma_n) * cos(alpha_n), -cos(sigma_n) * sin(alpha_n), a_n * sin(sigma_n),
         0.0, sin(alpha_n), cos(alpha_n), d_n,
         0.0, 0.0, 0.0, 1.0;
    return b;
}

/* Multiply two elements of SE2 */
std::vector<std::vector<double>> Kinematics::multiplySE2(const std::vector<std::vector<double>> &SE21, 
                                                         const std::vector<std::vector<double>> &SE22) const{
    std::vector<std::vector<double>> matrix(3, std::vector<double>(3));    
    for (int row=0; row < 3; row++) {
        for (int col=0; col < 3; col++) {
            double sum = 0.0;           
            for (int inner = 0; inner < 3; inner++) {
                sum += SE21[row][inner] * SE22[inner][col];
            }
            matrix[row][col] = sum;                 
        }    
    }
    
    return matrix;
} 
 
}
