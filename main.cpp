#include "include/calculator.h"
#include "include/Aabb.h"
#include "include/constructor.h"

int main(int argc, char **argv) {
    // read file
    char *file0;
    char *file1;

    file0 = "/home/seeeagull/CLionProjects/CCD/off/bumpy.off";//argv[1];
    file1 = "/home/seeeagull/CLionProjects/CCD/off/bunny.off";//argv[2];

    // parse mesh && construct box
    std::vector<Aabb> boxes;
    parseMesh(file0, file1, boxes);

    int nbox = boxes.size();
    std::vector<int> result_list;
    run_ccd(boxes, nbox, result_list);
    return 0;
}