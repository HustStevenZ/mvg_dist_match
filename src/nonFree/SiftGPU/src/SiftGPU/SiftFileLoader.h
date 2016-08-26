//
// Created by Sanqian on 16/7/26.
//

#ifndef SIFTGPU_SIFTFILELOADER_H
#define SIFTGPU_SIFTFILELOADER_H

#include <string>
#include <vector>

class SiftFileLoader {

public:
    static void loadFile(std::string fileName,std::vector<float> &keyPointBuffer,std::vector<float> &descriptionBuffer,int &descp_per_point);
};


#endif //SIFTGPU_SIFTFILELOADER_H
