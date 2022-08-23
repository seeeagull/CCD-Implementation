//
// Created by seeeagull on 22-8-21.
//

#include <vector>
#include <string>

#include <igl/edges.h>
#include <igl/readOFF.h>
#include <igl/readOBJ.h>
#include <igl/readPLY.h>

#include "Eigen/Core"
#include "../include/Aabb.h"

float nextUp(float x) {
    return nextafterf(x, x + std::numeric_limits<float>::max());
}

float nextDown(float x) {
    return nextafterf(x, x + std::numeric_limits<float>::max());
}

void addVertices(const Eigen::MatrixXd &vertices, std::vector<Aabb> &boxes) {
    int offset = boxes.size();
    boxes.resize(offset + vertices.rows());

    int size = vertices.rows();
#pragma omp parallel for default(none) shared(boxes, vertices, size, offset)
    for (int i = 0; i < size; i++) {
        boxes[offset + i].setId(offset + i);
        boxes[offset + i].setVertexIds({i, -i - 1, -i - 1});

        Eigen::Array3d v0 = vertices.row(i);

        boxes[offset + i].setAa(v0.unaryExpr(&nextDown));
        boxes[offset + i].setBb(v0.unaryExpr(&nextUp));
        boxes[offset + i].calMid();
    }
}

void addEdges(const std::vector<Aabb> &vertex_boxes,
              const Eigen::MatrixXi &edges, std::vector<Aabb> &boxes) {
    int offset = boxes.size();
    boxes.resize(offset + edges.rows());

    int size = edges.rows();
#pragma omp parallel for default(none) shared(boxes, vertex_boxes, edges, size, offset)
    for (int i = 0; i < size; i++) {
        boxes[offset + i].setId(offset + i);
        boxes[offset + i].setVertexIds(
                {edges(i, 0), edges(i, 1), -edges(i, 0) - 1});

        const Aabb &v0_box = vertex_boxes[edges(i, 0)];
        const Aabb &v1_box = vertex_boxes[edges(i, 1)];

        boxes[offset + i].setAa(v0_box.getAa().min(v1_box.getAa()));
        boxes[offset + i].setBb(v0_box.getBb().max(v1_box.getBb()));
        boxes[offset + i].calMid();
    }
}

void addFaces(const std::vector<Aabb> &vertex_boxes,
              const Eigen::MatrixXi &faces, std::vector<Aabb> &boxes) {
    int offset = boxes.size();
    boxes.resize(offset + faces.rows());

    int size = faces.rows();
#pragma omp parallel for default(none) shared(boxes, vertex_boxes, faces, size, offset)
    for (int i = 0; i < size; i++) {
        boxes[offset + i].setId(offset + i);
        boxes[offset + i].setVertexIds(
                {faces(i, 0), faces(i, 1), faces(i, 2)});

        const Aabb &v0_box = vertex_boxes[faces(i, 0)];
        const Aabb &v1_box = vertex_boxes[faces(i, 1)];
        const Aabb &v2_box = vertex_boxes[faces(i, 2)];

        boxes[offset + i].setAa(v0_box.getAa().min(v1_box.getAa()).min(v2_box.getAa()));
        boxes[offset + i].setBb(v0_box.getBb().max(v1_box.getBb()).max(v2_box.getBb()));
        boxes[offset + i].calMid();
    }
}

void constructBoxes(const Eigen::MatrixXd &vertices,
                    const Eigen::MatrixXi &edges, const Eigen::MatrixXi &faces,
                    std::vector<Aabb> &boxes) {
    addVertices(vertices, boxes);
    addEdges(boxes, edges, boxes);
    addFaces(boxes, faces, boxes);
}

void parseMesh(const char *filet0, const char *filet1,
               std::vector<Aabb> &boxes) {
    Eigen::MatrixXd vertices0, vertices1;
    Eigen::MatrixXi edges0, edges1;
    Eigen::MatrixXi faces0, faces1;
    std::vector<Aabb> boxes_new;

    std::string fn(filet0);
    std::string ext = fn.substr(fn.rfind('.') + 1);

    if (ext == "off") {
        igl::readOFF(filet0, vertices0, faces0);
        igl::readOFF(filet1, vertices1, faces1);
    } else if (ext == "obj") {
        igl::readOBJ(filet0, vertices0, faces0);
        igl::readOBJ(filet1, vertices1, faces1);
    } else if (ext == "ply") {
        igl::readPLY(filet0, vertices0, faces0);
        igl::readPLY(filet1, vertices1, faces1);
    } else {
        return;
    }

    igl::edges(faces0, edges0);
    igl::edges(faces1, edges1);
    constructBoxes(vertices0, edges0, faces0, boxes);
    constructBoxes(vertices1, edges1, faces1, boxes_new);
    boxes.insert(boxes.cend(), boxes_new.cbegin(), boxes_new.cend());
}