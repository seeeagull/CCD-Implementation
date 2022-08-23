//
// Created by seeeagull on 22-8-21.
//
#include <iostream>
#include <vector>
#include <algorithm>

#include "Eigen/Core"
#include "../include/Aabb.h"

void run_broadphase(std::vector<Aabb> boxes, int &nbox,
                    std::vector<std::pair<int, int>> &overlaps) {
    Eigen::Array3f ave = Eigen::Array3f(0, 0, 0);
    Eigen::Array3f ave2 = Eigen::Array3f(0, 0, 0);
#pragma omp parallel for default(none) shared(boxes, nbox, ave, ave2)
    for (int i = 0; i < nbox; ++i) {
        ave += boxes[i].getMid();
        ave2 += boxes[i].getSquare();
    }
    ave /= nbox;
    ave2 /= nbox;
    Eigen::Array3f var = ave2 - ave * ave;
    int axis = var[0] > var[1] ? 0 : 1;
    axis = var[2] > var[axis] ? 2 : axis;

#pragma opm parallel for default(none) shared(boxes, nbox, axis)
    for (int i = 0; i < nbox; ++i) {
        boxes[i].calMainAxis(axis);
    }

    std::sort(boxes.begin(), boxes.end());

    std::vector<std::pair<int, int>> q[2];
    int now = 0;
#pragma omp parallel for default(none) shared(q, now, overlaps, boxes, nbox, axis)
    for (int i = 1; i < nbox; ++i) {
        if (boxes[i - 1].getBb()[axis] > boxes[i].getAa()[axis]) {
            #pragma omp critical
            {
                q[now].emplace_back(i - 1, i);
            }
        }
    }

    int size;
    while (!q[now].empty()) {
        q[1 - now].clear();
        size = q[now].size();
#pragma omp parallel for default(none) shared(q, now, overlaps, boxes, nbox, size, axis)
        for (int j = 0; j < size; ++j) {
            std::pair<int, int> pr = q[now][j];
            if (boxes[pr.first].overlap(boxes[pr.second])) {
                #pragma omp critical
                {
                    overlaps.push_back(pr);
                }
            }
            if (pr.second < nbox - 1 &&
                    boxes[pr.first].getBb()[axis] > boxes[pr.second + 1].getAa()[axis]) {
                #pragma omp critical
                {
                    q[1 - now].emplace_back(pr.first, pr.second + 1);
                }
            }
        }
        now = 1 - now;
    }

    std::cout << "detected " << overlaps.size() << " collisions." << std::endl;
    /*for (auto & i : overlaps) {
        std::cout << i.first << ", " << i.second << ";";
    }*/
}

/*void run_narrowphase(std::vector<Aabb> boxes, int &nbox,
                     std::vector<std::pair<int, int>> overlaps) {
}*/

void run_ccd(std::vector<Aabb> boxes, int &nbox, std::vector<int> &result_list) {
    std::vector<std::pair<int, int>> overlaps;
    run_broadphase(boxes, nbox, overlaps);
    //run_narrowphase(boxes, nbox, overlaps);
}