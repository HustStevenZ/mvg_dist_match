//
// Created by Sanqian on 16/7/25.
//

#ifndef SIFTGPU_SIFTCONSOLE_H
#define SIFTGPU_SIFTCONSOLE_H

class SiftParam;
class SiftGPUEX;

class SiftConsole {
public:
    SiftConsole(int argc, char** argv);
    ~SiftConsole();
    void RunSiftGPU(int argc, char **argv);

//    void initGL();
    void ParseSiftParam(int argc, char** argv);
private:



    SiftGPUEX* _sift;
};


#endif //SIFTGPU_SIFTCONSOLE_H
