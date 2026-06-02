#pragma once

#include <sofa/pointcloud/config.h>
#include <sofa/pointcloud/components/PointCloudRendererBackend.h>
#include <sofa/pointcloud/components/liteviz-dataloader.h>
#include <sofa/component/visual/BaseCamera.h>

using sofa::component::visual::BaseCamera;
struct Splat {
    sofa::type::Vec3 position;
    int indice;

    Splat(const sofa::type::Vec3& p, const int indice)
        : position(p), indice(indice){}
};

struct CameraView {
    sofa::type::Vec3 position;
    sofa::type::Quat<double> orientation;
    double fov;
    double near;
    double far;
    float aspect;

    std::array<Plane, 6> clipPlanes;

    CameraView() : position(0,0,0), orientation(1,0,0,0), fov(0), near(0), far(0), aspect(0) {}
    CameraView(sofa::type::Vec3f position, 
        sofa::type::Quat<double> orientation, 
        double fov, 
        double near, 
        double far, 
        float  aspect):
            position(position), 
            orientation(orientation), 
            fov(fov), 
            near(near), 
            far(far),
            aspect(aspect) {
        double fovRad = fov * M_PI / 180.0;
        double tanHalfFov = std::tan(fovRad * 0.5);
        double hTanHalfFov = tanHalfFov * aspect;
        
        sofa::type::Vec3 forward = orientation.rotate(sofa::type::Vec3(0, 0, -1));
        sofa::type::Vec3 up      = orientation.rotate(sofa::type::Vec3(0, 1, 0));
        sofa::type::Vec3 right   = orientation.rotate(sofa::type::Vec3(1, 0, 0));
        forward.normalize();
        up.normalize();
        right.normalize();

        // Near and Far
        clipPlanes[0] = Plane(forward,  position + forward * near); 
        clipPlanes[1] = Plane(-forward, position + forward * far);

        // Top 
        sofa::type::Vec3 topNormal = forward * tanHalfFov - up;
        clipPlanes[2] = Plane(topNormal, position);

        // Bottom
        sofa::type::Vec3 bottomNormal = forward * tanHalfFov + up;
        clipPlanes[3] = Plane(bottomNormal, position);

        // Left
        sofa::type::Vec3 leftNormal = forward * hTanHalfFov + right;
        clipPlanes[4] = Plane(leftNormal, position);

        // Right
        sofa::type::Vec3 rightNormal = forward * hTanHalfFov - right;
        clipPlanes[5] = Plane(rightNormal, position);
    }
    ~CameraView() {}

    bool contains(const Splat* splat) const;
};

struct Cube {
    sofa::type::Vec3f min;
    sofa::type::Vec3f max;

    Cube() : min(0,0,0), max(0,0,0) {} 
    Cube(const sofa::type::Vec3f& min, const sofa::type::Vec3f& max) : min(min), max(max) {}

    bool contains(const Splat* splat) const;

    bool intersects(const Cube& other) const;

    bool intersects(const CameraView& camera) const;

    int getLod(const CameraView& camera, const sofa::type::vector<float>* distances) const;
    void draw(const sofa::core::visual::VisualParams* vparams, sofa::type::RGBAColor color); 

    inline friend std::istream& operator >> ( std::istream& in, Cube& s ){
        in >> s.min >> s.max;
        return in;
    }

    inline friend std::ostream& operator << ( std::ostream& out, const Cube& s ){
        out << s.min << " " << s.max;
        return out;
    }
};

struct QuadTreeNode {
    Cube cube;
    int maxSplats;

    std::map<int, sofa::type::vector<Splat*>> lodPoints;
    const sofa::type::vector<float>* distances = nullptr;

    bool isSubdivided = false;
    QuadTreeNode* upNorthWest = nullptr;
    QuadTreeNode* upNorthEast = nullptr;
    QuadTreeNode* upSouthWest = nullptr;
    QuadTreeNode* upSouthEast = nullptr;


    QuadTreeNode* downNorthWest = nullptr;
    QuadTreeNode* downNorthEast = nullptr;
    QuadTreeNode* downSouthWest = nullptr;
    QuadTreeNode* downSouthEast = nullptr;

    QuadTreeNode(const sofa::type::vector<float>* distances) : distances(distances){}
    ~QuadTreeNode(){
        distances = nullptr;
        

        if (isSubdivided) {
            delete upNorthWest;
            upNorthWest = nullptr;
            delete upNorthEast;
            upNorthEast = nullptr;
            delete upSouthWest;
            upSouthWest = nullptr;
            delete upSouthEast;
            upSouthEast = nullptr;

            delete downNorthWest;
            downNorthWest = nullptr;
            delete downNorthEast;
            downNorthEast = nullptr;
            delete downSouthWest;
            downSouthWest = nullptr;
            delete downSouthEast;
            downSouthEast = nullptr;
        }
    }

    bool insertSplat(Splat* splat, int lod);
    
    void subdivide();
    void query(CameraView& camera, std::list<int>& results);
    void draw(const sofa::core::visual::VisualParams* vparams, sofa::type::RGBAColor color); 
    void drawIntersec(const sofa::core::visual::VisualParams* vparams, sofa::type::RGBAColor color, CameraView& camera);
};
