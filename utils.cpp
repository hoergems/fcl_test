#include "utils.hpp"
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

using std::cout;
using std::endl;

namespace utils {

std::vector<fcl::OBB> Utils::createManipulatorCollisionStructures(const std::vector<double> &joint_angles, 
                                                                  std::shared_ptr<shared::Kinematics> &kinematics){
    fcl::AABB link_aabb(fcl::Vec3f(0.0, -0.0025, -0.0025), fcl::Vec3f(1.0, 0.0025, 0.0025));
    std::vector<fcl::OBB> collision_structures;
    int n = 0;
    for (size_t i = 0; i < joint_angles.size(); i++) {
        const std::pair<fcl::Vec3f, fcl::Matrix3f> pose_link_n = kinematics->getPoseOfLinkN(joint_angles, n);
        fcl::OBB obb;
        fcl::convertBV(link_aabb, fcl::Transform3f(pose_link_n.second, pose_link_n.first), obb);
        collision_structures.push_back(obb);
        n++;
    }
    
    return collision_structures;
}

template<class T>
struct VecToList
{
    static PyObject* convert(const std::vector<T>& vec)
    {
        boost::python::list* l = new boost::python::list();
        for(size_t i = 0; i < vec.size(); i++)
            (*l).append(vec[i]);

        return l->ptr();
    }
};

BOOST_PYTHON_MODULE(util) {
    // An established convention for using boost.python.
    using namespace boost::python;
    
    typedef std::vector<std::vector<double> > vec_vec;
    
    class_<std::vector<std::vector<double> > > ("v2_double")
         .def(vector_indexing_suite<std::vector<std::vector<double> > >());
    
    class_<std::vector<std::vector<int> > > ("v2_int")
         .def(vector_indexing_suite<std::vector<std::vector<int> > >());
         
    class_<std::vector<double> > ("v_double")
         .def(vector_indexing_suite<std::vector<double> >());
         
    class_<std::vector<int> > ("v_int")
         .def(vector_indexing_suite<std::vector<int> >());
    
    class_<fcl::OBB>("OBB");
    to_python_converter<std::vector<fcl::OBB, std::allocator<fcl::OBB> >, VecToList<fcl::OBB> >();
    class_<Utils>("Utils")
         .def("createManipulatorCollisionStructures", &Utils::createManipulatorCollisionStructures)         
         
    ;
}

}
