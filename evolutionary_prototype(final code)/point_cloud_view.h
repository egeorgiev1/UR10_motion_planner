#pragma once

#include <array>
#include <vector>
#include <string>

#include "Ogre.h"

#include "util.h"

typedef
    std::vector<
        std::array<real_t, 3>
    >
    points_array_t;

class PointCloudView {
private:
    std::string _uuid;

    Ogre::SceneManager* _scene_manager;

    std::vector<Ogre::SceneNode*> _available_spheres;
    std::vector<Ogre::SceneNode*> _in_use_spheres;

public:
    PointCloudView(
        std::string uuid,
        Ogre::SceneManager* scene_manager
    );

    void set_points(const points_array_t& points);

    void set_visibility(bool is_visible);
};
