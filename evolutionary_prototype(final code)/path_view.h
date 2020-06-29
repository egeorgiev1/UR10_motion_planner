#pragma once

#include <array>
#include <vector>
#include <string>

#include <Ogre.h>

#include "util.h"

class PathView {
private:
    std::string _uuid;

    Ogre::SceneManager* _scene_manager { nullptr };
    
    Ogre::SceneNode* _scene_node { nullptr };
    Ogre::ColourValue _edge_color { 0, 0, 0, 0 };

    Ogre::MaterialPtr _create_path_material();

public:
    PathView(
        std::string uuid,
        Ogre::SceneManager* scene_manager,
        Ogre::ColourValue edge_color
    );

    void set_points(const std::vector<std::array<real_t, 3>>& points);
};