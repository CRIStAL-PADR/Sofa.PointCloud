#include <sofa/pointcloud/fwd.h>

#include <sofa/pointcloud/components/PointCloudGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::core
{

template<>
void registerToFactory<sofa::pointcloud::components::PointCloudGeometry>(sofa::core::ObjectFactory* factory){
    factory->registerObjects(sofa::core::ObjectRegistrationData("Store a point cloud.")
                             .add< sofa::pointcloud::components::PointCloudGeometry >());
}

}

namespace sofa::core::objectmodel
{

/// Specialization for reading strings
template<>
bool Data<Eigen::MatrixXf>::read( const std::string& str ){}

/// Specialization for reading strings
template<>
std::string Data<Eigen::MatrixXf>::getValueString() const
{
    std::stringstream tmp;
    tmp << "EigenMatrix(" << m_value.getValue().rows() << "," << m_value.getValue().cols() << ")";
    return tmp.str();
}

}

namespace sofa::pointcloud::components {

    PointCloudGeometry::PointCloudGeometry():
    d_positions(initData(&d_positions, "positions", "Indices")), 
    d_orientations(initData(&d_orientations, "orientations", "Indices")), 
    d_scales(initData(&d_scales, "scales", "")), 
    d_opacities(initData(&d_opacities, "opacities", "")), 
    d_sphericalHarmonics(initData(&d_sphericalHarmonics, "sphericalHarmonics", "")), 
    d_indices(initData(&d_indices, "indices", "Indices")) {
        addUpdateCallback("updateBox", {&d_positions}, [this](const sofa::core::DataTracker& tracker)
        {
            updateBBox();
            return core::objectmodel::ComponentState::Valid;
        },{&f_bbox});
    }

    PointCloudGeometry::~PointCloudGeometry() {}



    void loader(std::atomic<bool>& running, const std::string& name) {
        std::vector<std::string> spinner = {"⠁","⠂","⠄","⡀","⢀","⠠","⠐","⠈"};
        int i = 0;

        std::cout << "\033[?25l"; // hide
        while (running) {
            std::cout << "\rLoading "<< name <<": " << spinner[i % spinner.size()] << std::flush;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            ++i;
        }
        std::cout << "\033[?25h"; // show
    }


    void PointCloudGeometry::updateBBox() {
        if(isComponentStateInvalid())
            return;

        type::BoundingBox box;
        auto points = data->xyz;
        for(int i=0;i<points.rows();++i)
        {
            auto point = points.row(i);
            box.include(type::Vec3{point.x(), point.y(), point.z()});
        }
        f_bbox.setValue(box);
    }


    size_t PointCloudGeometry::size() {
        if(isComponentStateInvalid())
            return 0;

        return data->xyz.rows();
    }

    void PointCloudGeometry::updateDataFields() {
        if(isComponentStateInvalid())
            return;

        d_positions.setValue(data->xyz);
        d_orientations.setValue(data->rot);
        d_scales.setValue(data->scale);
        d_componentState = core::objectmodel::ComponentState::Valid;
    }

}

