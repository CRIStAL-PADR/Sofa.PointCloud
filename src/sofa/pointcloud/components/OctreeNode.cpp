#include <sofa/pointcloud/fwd.h>
#include <sofa/pointcloud/components/OctreeNode.h>
#include <sofa/core/visual/VisualParams.h>

// Cube
bool Cube::contains(const Splat* splat) const {
    return min.x() <= splat->position.x() && max.x() >= splat->position.x()
        && min.y() <= splat->position.y() && max.y() >= splat->position.y()
        && min.z() <= splat->position.z() && max.z() >= splat->position.z();
}

bool Cube::intersects(const Cube& other) const {
    return min.x() <= other.max.x() && max.x() >= other.min.x()
        && min.y() <= other.max.y() && max.y() >= other.min.y()
        && min.z() <= other.max.z() && max.z() >= other.min.z();
}

bool Cube::intersects(const CameraView& camera) const {
    for (int i = 0; i < 6; ++i) {
        const Plane& p = camera.clipPlanes[i];
        sofa::type::Vec3f pVertex;
        pVertex[0] = (p.normal[0] >= 0) ? max[0] : min[0];
        pVertex[1] = (p.normal[1] >= 0) ? max[1] : min[1];
        pVertex[2] = (p.normal[2] >= 0) ? max[2] : min[2];
        if (sofa::type::dot(p.normal, pVertex)  < p.d) {
            return false;
        }
    }
    return true;
}

void Cube::draw(const sofa::core::visual::VisualParams* vparams, sofa::type::RGBAColor color) {
    auto* drawer = vparams->drawTool();

    float xmin = min.x();
    float ymin = min.y();
    float zmin = min.z();
    float xmax = max.x();
    float ymax = max.y();
    float zmax = max.z();

    drawer->drawLine(sofa::type::Vec3f(xmin, ymin, zmin), sofa::type::Vec3f(xmax, ymin, zmin), color);
    drawer->drawLine(sofa::type::Vec3f(xmax, ymin, zmin), sofa::type::Vec3f(xmax, ymax, zmin), color);
    drawer->drawLine(sofa::type::Vec3f(xmax, ymax, zmin), sofa::type::Vec3f(xmin, ymax, zmin), color);
    drawer->drawLine(sofa::type::Vec3f(xmin, ymax, zmin), sofa::type::Vec3f(xmin, ymin, zmin), color);

    drawer->drawLine(sofa::type::Vec3f(xmin, ymin, zmax), sofa::type::Vec3f(xmax, ymin, zmax), color);
    drawer->drawLine(sofa::type::Vec3f(xmax, ymin, zmax), sofa::type::Vec3f(xmax, ymax, zmax), color);
    drawer->drawLine(sofa::type::Vec3f(xmax, ymax, zmax), sofa::type::Vec3f(xmin, ymax, zmax), color);        
    drawer->drawLine(sofa::type::Vec3f(xmin, ymax, zmax), sofa::type::Vec3f(xmin, ymin, zmax), color);

    drawer->drawLine(sofa::type::Vec3f(xmin, ymin, zmin), sofa::type::Vec3f(xmin, ymin, zmax), color);
    drawer->drawLine(sofa::type::Vec3f(xmax, ymin, zmin), sofa::type::Vec3f(xmax, ymin, zmax), color);
    drawer->drawLine(sofa::type::Vec3f(xmax, ymax, zmin), sofa::type::Vec3f(xmax, ymax, zmax), color);
    drawer->drawLine(sofa::type::Vec3f(xmin, ymax, zmin), sofa::type::Vec3f(xmin, ymax, zmax), color);  

    drawer->drawLine(sofa::type::Vec3f(xmin, ymin, zmin), sofa::type::Vec3f(xmin, ymax, zmin), color);
    drawer->drawLine(sofa::type::Vec3f(xmin, ymin, zmax), sofa::type::Vec3f(xmin, ymax, zmax), color);
    drawer->drawLine(sofa::type::Vec3f(xmax, ymin, zmin), sofa::type::Vec3f(xmax, ymax, zmin), color);        
    drawer->drawLine(sofa::type::Vec3f(xmax, ymin, zmax), sofa::type::Vec3f(xmax, ymax, zmax), color);
    
}

int Cube::getLod(const CameraView& camera, const sofa::type::vector<float>* distances) const {
    auto center = (min + max) / 2;
    auto distance = (camera.position - center).norm();
    for (int i = 0; i < distances->size(); ++i) {
        if (distance < (*distances)[i]) return i;
    }
    return -1;
}

// CameraView
bool CameraView::contains(const Splat* splat) const {
    if (splat == nullptr) return false;
    for (const auto& plane : clipPlanes) {
        if (sofa::type::dot(plane.normal, splat->position) < plane.d) {
            return false;
        }
    }
    return true;
}

// QuadTreeNode
bool QuadTreeNode::insertSplat(Splat* splat, int lod){
    if (!cube.contains(splat))
        return false;

    if (!isSubdivided && lodPoints[lod].size() < maxSplats) {
        lodPoints[lod].push_back(splat);
        return true;
    }

    if (!isSubdivided) 
        subdivide();

    if (upNorthWest->insertSplat(splat, lod))
        return true;
    if (upNorthEast->insertSplat(splat, lod))
        return true;
    if (upSouthWest->insertSplat(splat, lod))
        return true;
    if (upSouthEast->insertSplat(splat, lod))
        return true;

    if (downNorthWest->insertSplat(splat, lod))
        return true;
    if (downNorthEast->insertSplat(splat, lod))
        return true;
    if (downSouthWest->insertSplat(splat, lod))
        return true;
    if (downSouthEast->insertSplat(splat, lod))
        return true;
    return false;
}

void QuadTreeNode::subdivide(){
    isSubdivided = true;
    auto x_half = (cube.min.x() + cube.max.x()) / 2;
    auto y_half = (cube.min.y() + cube.max.y()) / 2;
    auto z_half = (cube.min.z() + cube.max.z()) / 2;

    upNorthWest = new QuadTreeNode(distances);
    auto min = sofa::type::Vec3f(cube.min.x(), y_half, z_half);
    auto max = sofa::type::Vec3f(x_half, cube.max.y(), cube.max.z());
    upNorthWest->cube = Cube(min, max);
    upNorthWest->maxSplats = maxSplats;

    upNorthEast = new QuadTreeNode(distances);
    min = sofa::type::Vec3f(x_half, y_half, z_half);
    max = sofa::type::Vec3f(cube.max.x(), cube.max.y(), cube.max.z());
    upNorthEast->cube = Cube(min, max);
    upNorthEast->maxSplats = maxSplats;

    upSouthWest = new QuadTreeNode(distances);
    min = sofa::type::Vec3f(cube.min.x(), cube.min.y(), z_half);
    max = sofa::type::Vec3f(x_half, y_half, cube.max.z());
    upSouthWest->cube = Cube(min, max);
    upSouthWest->maxSplats = maxSplats;

    upSouthEast = new QuadTreeNode(distances);
    min = sofa::type::Vec3f(x_half, cube.min.y(), z_half);
    max = sofa::type::Vec3f(cube.max.x(), y_half, cube.max.z());
    upSouthEast->cube = Cube(min, max);
    upSouthEast->maxSplats = maxSplats;

    // Down
    downNorthWest = new QuadTreeNode(distances);
    min = sofa::type::Vec3f(cube.min.x(), y_half, cube.min.z());
    max = sofa::type::Vec3f(x_half, cube.max.y(), z_half);
    downNorthWest->cube = Cube(min, max);
    downNorthWest->maxSplats = maxSplats;

    downNorthEast = new QuadTreeNode(distances);
    min = sofa::type::Vec3f(x_half, y_half, cube.min.z());
    max = sofa::type::Vec3f(cube.max.x(), cube.max.y(), z_half);
    downNorthEast->cube = Cube(min, max);
    downNorthEast->maxSplats = maxSplats;

    downSouthWest = new QuadTreeNode(distances);
    min = sofa::type::Vec3f(cube.min.x(), cube.min.y(), cube.min.z());
    max = sofa::type::Vec3f(x_half, y_half, z_half);
    downSouthWest->cube = Cube(min, max);
    downSouthWest->maxSplats = maxSplats;

    downSouthEast = new QuadTreeNode(distances);
    min = sofa::type::Vec3f(x_half, cube.min.y(), cube.min.z());
    max = sofa::type::Vec3f(cube.max.x(), y_half, z_half);
    downSouthEast->cube = Cube(min, max);
    downSouthEast->maxSplats = maxSplats;

    for (auto it = lodPoints.begin(); it != lodPoints.end(); ++it) {
        int lodValue = it->first; 
        auto& splatVec = it->second;
        for (Splat* s : splatVec) {
            if (s != nullptr) {
                insertSplat(s, lodValue);
            }
        }
        splatVec.clear(); 
    }
    lodPoints.clear();
}

void QuadTreeNode::query(CameraView& camera, std::list<int>& results){
    if (!cube.intersects(camera))
        return;
    if  (isSubdivided) {
        upNorthWest->query(camera, results);
        upNorthEast->query(camera, results);
        upSouthWest->query(camera, results);
        upSouthEast->query(camera, results);

        downNorthWest->query(camera, results);
        downNorthEast->query(camera, results);
        downSouthWest->query(camera, results);
        downSouthEast->query(camera, results);
        return;
    }

    int lod = cube.getLod(camera, distances);
    if (lod == -1) return;
    auto splats = lodPoints.find(lod);
    if (splats != lodPoints.end()) {
        for (Splat* splat : splats->second) {    
            if (camera.contains(splat)) {
                results.push_back(splat->indice);
            }
        }
    }

}

void QuadTreeNode::draw(const sofa::core::visual::VisualParams* vparams, sofa::type::RGBAColor color) {
    auto* drawer = vparams->drawTool();

    cube.draw(vparams, color);

    if (isSubdivided) {
        if (upNorthWest) upNorthWest->draw(vparams, color);
        if (upNorthEast) upNorthEast->draw(vparams, color);
        if (upSouthWest) upSouthWest->draw(vparams, color);
        if (upSouthEast) upSouthEast->draw(vparams, color);
        if (downNorthWest) downNorthWest->draw(vparams, color);
        if (downNorthEast) downNorthEast->draw(vparams, color);
        if (downSouthWest) downSouthWest->draw(vparams, color);
        if (downSouthEast) downSouthEast->draw(vparams, color);
    }
}

void QuadTreeNode::drawIntersec(const sofa::core::visual::VisualParams* vparams, sofa::type::RGBAColor color, CameraView& camera) {
    auto* drawer = vparams->drawTool();
    if (!cube.intersects(camera))
        return;
    cube.draw(vparams, color);

    if (isSubdivided) {
        if (upNorthWest) upNorthWest->drawIntersec(vparams, color, camera);
        if (upNorthEast) upNorthEast->drawIntersec(vparams, color, camera);
        if (upSouthWest) upSouthWest->drawIntersec(vparams, color, camera);
        if (upSouthEast) upSouthEast->drawIntersec(vparams, color, camera);
        if (downNorthWest) downNorthWest->drawIntersec(vparams, color, camera);
        if (downNorthEast) downNorthEast->drawIntersec(vparams, color, camera);
        if (downSouthWest) downSouthWest->drawIntersec(vparams, color, camera);
        if (downSouthEast) downSouthEast->drawIntersec(vparams, color, camera);
    }
}