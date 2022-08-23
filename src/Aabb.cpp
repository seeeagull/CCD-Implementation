//
// Created by seeeagull on 22-8-21.
//

#include "../include/Aabb.h"

#include <utility>
#include "Eigen/Core"

Aabb::Aabb() {
    m_id = -1;
    m_vertexIds = Eigen::Array3i(-1, -1, -1);
    m_aa = Eigen::Array3f(0, 0, 0);
    m_bb = Eigen::Array3f(0, 0, 0);
    m_mid = Eigen::Array3f(0, 0, 0);
    m_mainAxis = m_mid[0];
}

Aabb::Aabb(int id, Eigen::Array3i vertexIds, Eigen::Array3f aa, Eigen::Array3f bb) {
    m_id = id;
    m_vertexIds = std::move(vertexIds);
    m_aa = std::move(aa);
    m_bb = std::move(bb);
    m_mid = (m_aa + m_bb) * 0.5;
    m_mainAxis = m_mid[0];
}

Eigen::Array3f Aabb::getAa() const {
    return m_aa;
}

Eigen::Array3f Aabb::getBb() const {
    return m_bb;
}

Eigen::Array3f Aabb::getMid() const {
    return m_mid;
}

Eigen::Array3f Aabb::getSquare() const {
    return m_mid * m_mid;
}

double Aabb::getMainAxis() const {
    return m_mainAxis;
}

void Aabb::setId(int id) {
    m_id = id;
}

void Aabb::setVertexIds(Eigen::Array3i vertexIds) {
    m_vertexIds = std::move(vertexIds);
}

void Aabb::setAa(Eigen::Array3f aa) {
    m_aa = std::move(aa);
}

void Aabb::setBb(Eigen::Array3f bb) {
    m_bb = std::move(bb);
}

void Aabb::calMid() {
    m_mid = (m_aa + m_bb) * 0.5;
}

void Aabb::calMainAxis(int axis) {
    m_mainAxis = m_mid[axis];
}

bool Aabb::overlap(Aabb obj) {
    Eigen::Array3f obj_aa = obj.getAa();
    Eigen::Array3f obj_bb = obj.getBb();
    return !(m_bb[0] < obj_aa[0] || obj_bb[0] < m_aa[0] ||
             m_bb[1] < obj_aa[1] || obj_bb[1] < m_aa[1] ||
             m_bb[2] < obj_aa[2] || obj_bb[2] < m_aa[2]);
}

bool Aabb::isVertex() {
    return m_vertexIds.y() < 0 && m_vertexIds.z() < 0;
}

bool Aabb::isEdge() {
    return m_vertexIds.y() >= 0 && m_vertexIds.z() < 0;
}

bool Aabb::isFace() {
    return m_vertexIds.z() >= 0;
}
