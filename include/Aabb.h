//
// Created by seeeagull on 22-8-21.
//

#ifndef CCD_AABB_H
#define CCD_AABB_H

#include "Eigen/Core"

class Aabb {
public:
    Aabb();
    Aabb(int id, Eigen::Array3i vertexId, Eigen::Array3f aa, Eigen::Array3f bb);
    bool operator < (Aabb const &obj) const {
        return m_mainAxis < obj.getMainAxis();
    }
    bool operator > (Aabb const &obj) const {
        return m_mainAxis > obj.getMainAxis();
    }
    bool operator == (Aabb const &obj) const {
        return m_mainAxis == obj.getMainAxis();
    }
    Eigen::Array3f getAa() const;
    Eigen::Array3f getBb() const;
    Eigen::Array3f getMid() const;
    Eigen::Array3f getSquare() const;
    double getMainAxis() const;
    void setId(int id);
    void setVertexIds(Eigen::Array3i vertexIds);
    void setAa(Eigen::Array3f aa);
    void setBb(Eigen::Array3f bb);
    void calMid();
    void calMainAxis(int axis);
    bool overlap(Aabb obj);
    bool isVertex();
    bool isEdge();
    bool isFace();
private:
    int m_id;
    Eigen::Array3i m_vertexIds;
    Eigen::Array3f m_aa;
    Eigen::Array3f m_bb;
    Eigen::Array3f m_mid;
    double m_mainAxis;
};


#endif //CCD_AABB_H
