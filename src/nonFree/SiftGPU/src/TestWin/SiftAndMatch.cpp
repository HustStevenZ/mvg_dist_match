//
// Created by Sanqian on 16/7/27.
//

#include "SiftAndMatch.h"
#include <iostream>
#include <SiftGPU.h>
#include <vector>

#include <SiftFileLoader.h>
#ifdef __linux__
#include <GL/glut.h>

#endif
#ifdef __APPLE__

#include <OpenGL/gl.h>
#include <GLUT/glut.h>

#endif
using namespace std;
int main(int argc,char** argv)
{

    SiftGPU* sift = new SiftGPU();
    glutInit(&argc,argv);
    int id;
    int num1,num2;
    vector<SiftGPU::SiftKeypoint> key1;
    vector<float> description1;
    vector<SiftGPU::SiftKeypoint> key2;
    vector<float> description2;
    glutInitDisplayMode (GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize (0,0);
    glutInitWindowPosition(0,0);
    id = glutCreateWindow ("SIFT_GPU");

    sift->ParseParam(2,argv);
    if(sift->RunSIFT(argv[2]))
    {
        num1 = sift->GetFeatureNum();
        key1.resize(num1);
        description1.resize(num1*128);
        sift->GetFeatureVector(&key1[0],&description1[0]);
    }

    if(sift->RunSIFT(argv[3]))
    {
        num2 = sift->GetFeatureNum();
        key2.resize(num2);
        description2.resize(128*num2);
        sift->GetFeatureVector(&key2[0],&description2[0]);
    }

    std::vector<float> keypoint0;
    std::vector<float> loaddescription0;

    std::vector<float> keypoint1;
    std::vector<float> loaddescription1;
    int descip_per_point0 =0;
    int descip_per_point1 =0;
    string filename1(argv[2]);
    string filename2(argv[3]);
//    SiftFileLoader::loadFile(filename1+".sift",keypoint0,loaddescription0,descip_per_point0);
//    SiftFileLoader::loadFile(filename2+".sift",keypoint1,loaddescription1,descip_per_point1);
    SiftMatchGPU* matchGPU = new SiftMatchGPU(std::max(key1.size()/4,key2.size()/4));
//        glutInit(&argc,argv);
//    matchGPU->CreateContextGL();
    matchGPU->VerifyContextGL();
    matchGPU->SetDescriptors(0,key1.size()/4,&description1[0]);
    matchGPU->SetDescriptors(1,key2.size()/4,&description2[0]);

    int (*match_buf)[2] = new int[std::max(key1.size()/4,key2.size()/4)][2];
    //use the default thresholds. Check the declaration in SiftGPU.h
    int num_match = matchGPU->GetSiftMatch(std::max(key1.size()/4,key2.size()/4), match_buf);
    std::cout << num_match << " sift matches were found;\n";

//    SiftMatchGPU* matchGPU = new SiftMatchGPU();
}