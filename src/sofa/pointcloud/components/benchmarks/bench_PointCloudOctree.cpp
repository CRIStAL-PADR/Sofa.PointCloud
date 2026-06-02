#include <benchmark/benchmark.h>
#include <sofa/pointcloud/components/PointCloudOctree.h>
#include <sofa/component/visual/BaseCamera.h>
#include <sofa/component/visual/InteractiveCamera.h>
#include <sofa/type/Vec.h>
#include <fstream>
#include <random>

using namespace sofa::pointcloud::components;
using sofa::component::visual::BaseCamera;
using sofa::component::visual::InteractiveCamera;


class PointCloudOctreeFixture : public benchmark::Fixture {
    
public:
    PointCloudOctree* octree = nullptr;
    sofa::core::objectmodel::New<sofa::component::visual::InteractiveCamera> camera;
    CameraView view;

    // S'exécute avant chaque test
    void SetUp(const ::benchmark::State& state) {
        octree = new PointCloudOctree();
        octree->d_filenames.setValue(std::vector<std::string>({"splats/spot/spot.ply"}));
        octree->d_maxSplats.setValue(100);
        octree->d_loadingLog.setValue(false);
        octree->init();

        camera = sofa::core::objectmodel::New<InteractiveCamera>();
        camera->d_position.setValue({0, 0, 25});
        camera->d_lookAt.setValue({0, 0, 0});
        camera->d_fieldOfView.setValue(45.0f);
        camera->d_zNear.setValue(0.1f);
        camera->d_zFar.setValue(1000.0f);

        sofa::type::Vec3 position = camera->getPosition();
        sofa::type::Quat direction = camera->getOrientation();
        auto fov = camera->getFieldOfView();
        auto near = camera->getZNear();
        auto far = camera->getZFar();

        view = CameraView(position,
            direction,
            fov,
            near,
            far,
            1.33f);

    }


    // S'exécute après chaque test
    void TearDown(const ::benchmark::State& state) {
        if (octree != nullptr) {
            delete octree;
            octree = nullptr;
        }
    }

};

BENCHMARK_F(PointCloudOctreeFixture, updateIndices)(benchmark::State& st) {
    for (auto _ : st) {
        octree->updateIndices(camera.get(), 1.33f);
        auto pos = camera->d_position.getValue();
        camera->d_position.setValue(sofa::type::Vec3(pos[0], pos[1], pos[2] + 0.01));
    }   
}


BENCHMARK_F(PointCloudOctreeFixture, getLod)(benchmark::State& st) {
    for (auto _ : st) {
        int index = std::rand() % octree->d_positions.getValue().rows();
        octree->getLod(index);
    }   
}

BENCHMARK_F(PointCloudOctreeFixture, query)(benchmark::State& st) {
    for (auto _ : st) {
        std::list<int> indices;
        octree->ocTree->query(view, indices);
    }   
}

BENCHMARK_MAIN();