//
// Created by seeeagull on 22-8-21.
//
#pragma once

#ifndef CCD_CONSTRUCTOR_H
#define CCD_CONSTRUCTOR_H

#include <vector>

#include "Aabb.h"

float nextUp(float x);

float nextDown(float x);

void addVertices(const Eigen::MatrixXd &vertices_t0,
                 const Eigen::MatrixXd &vertices_t1, std::vector<Aabb> &boxes);

void addEdges(const std::vector<Aabb> &vertex_boxes,
              const Eigen::MatrixXi &edges, std::vector<Aabb> &boxes);

void addFaces(const std::vector<Aabb> &vertex_boxes,
              const Eigen::MatrixXi &faces, std::vector<Aabb> &boxes);

void constructBoxes(const Eigen::MatrixXd &vertices,
                    const Eigen::MatrixXi &edges, const Eigen::MatrixXi &faces,
                    std::vector<Aabb> &boxes);

void parseMesh(const char *filet0, const char *filet1,
               std::vector<Aabb> &boxes);

#endif //CCD_CONSTRUCTOR_H
