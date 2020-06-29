#pragma once

#include <array>
#include <vector>
#include <string>

#include <Ogre.h>

#include "util.h"

struct GraphEdge {
    std::array<real_t, 3> start;
    std::array<real_t, 3> end;
};

class GraphView {
private:
    std::string _uuid;

    Ogre::SceneManager* _scene_manager { nullptr };
    
    Ogre::ManualObject* _graph_geometry { nullptr };
    Ogre::SceneNode* _scene_node { nullptr };
    Ogre::ColourValue _edge_color { 0, 0, 0, 0 };

    Ogre::ManualObject* _create_graph_geometry();

public:
    GraphView(
        std::string uuid,
        Ogre::SceneManager* scene_manager,
        Ogre::ColourValue edge_color
    );

    void set_edges(const std::vector<GraphEdge>& edge_list);

    void set_visibility(bool is_visible);
};