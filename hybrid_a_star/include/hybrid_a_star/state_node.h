

#ifndef HYBRID_A_STAR_STATE_NODE_H
#define HYBRID_A_STAR_STATE_NODE_H

#include "type.h"

#include <Eigen/Dense>
/*(x_，y_，phi_）点就是这一串点中的最后一个。
其实，这些点都是在1个grid内的。
即，1个grid包含1个Node3d（下文便以node指代），1个Node3d包含了以（x_，y_，phi_）为终点、同在1个grid内的、一串路径点集的信息。*/
struct StateNode {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum NODE_STATUS {
        NOT_VISITED = 0, IN_OPENSET = 1, IN_CLOSESET = 2
    };

    enum DIRECTION {
        FORWARD = 0, BACKWARD = 1, NO = 3
    };

    StateNode() = delete;

    explicit StateNode(const Vec3i &grid_index) {
        node_status_ = NOT_VISITED;
        grid_index_ = grid_index;
        parent_node_ = nullptr;
    }

    void Reset() {
        node_status_ = NOT_VISITED;
        parent_node_ = nullptr;
    }

    NODE_STATUS node_status_;
    DIRECTION direction_{};

    Vec3d state_;//Coordinate in gragh 
    Vec3i grid_index_;//x,y,phi是node的state grid index in pixel

    double g_cost_{}, f_cost_{};
    int steering_grade_{};

    StateNode *parent_node_;
    typedef StateNode *Ptr;

    VectorVec3d intermediate_states_;//node连接的一串点的坐标集合
};

#endif //HYBRID_A_STAR_STATE_NODE_H
