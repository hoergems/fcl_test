#include <memory>
#include "Kinematics.hpp"
#include <iostream> 
#include <fstream>
#include <boost/python.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp> 
#include "fcl/BV/BV.h"

using boost::property_tree::ptree;

namespace utils {

class Utils {
    public: 
        std::vector<fcl::OBB> createManipulatorCollisionStructures(const std::vector<double> &joint_angles, 
                                                                   std::shared_ptr<shared::Kinematics> &kinematics); 
};
           


}
