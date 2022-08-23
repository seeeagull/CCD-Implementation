//
// Created by seeeagull on 22-8-21.
//

#ifndef CCD_CALCULATOR_H
#define CCD_CALCULATOR_H

#include <vector>
#include <cstdlib>

#include "Eigen/Core"
#include "Aabb.h"

void run_broadphase(std::vector<Aabb> boxes, int &nbox,
                    std::vector<std::pair<int, int>> &overlaps);

void run_narrowphase();

void run_ccd(std::vector<Aabb> boxes, int &nbox, std::vector<int> &result_list);

#endif //CCD_CALCULATOR_H
