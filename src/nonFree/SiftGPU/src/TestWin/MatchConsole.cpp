//
// Created by Sanqian on 16/7/26.
//

#include "MatchConsole.h"
#include <SiftFileLoader.h>
#include <SiftGPU.h>
#include <math.h>
#include <iostream>
#ifdef __linux__

#include <GL/glut.h>
#endif

int main(int argc,char** argv)
{
    if(argc!=4)
        return -1;

    if(argc==4)
    {
        SiftGPU* sift = new SiftGPU();
        glutInit(&argc,argv);
        int id;
        glutInitDisplayMode (GLUT_RGBA | GLUT_DOUBLE);
        glutInitWindowSize (0,0);
        glutInitWindowPosition(0,0);
        id = glutCreateWindow ("SIFT_GPU");

        sift->ParseParam(2,argv);

        std::vector<float> keypoint0;
        std::vector<float> description0;

        std::vector<float> keypoint1;
        std::vector<float> description1;
        int descip_per_point0 =0;
        int descip_per_point1 =0;
        SiftFileLoader::loadFile(argv[2],keypoint0,description0,descip_per_point0);
        SiftFileLoader::loadFile(argv[3],keypoint1,description1,descip_per_point1);


        SiftMatchGPU* matchGPU = new SiftMatchGPU(std::max(keypoint0.size()/4,keypoint1.size()/4));
//        glutInit(&argc,argv);
        matchGPU->VerifyContextGL();
        matchGPU->SetDescriptors(0,keypoint0.size()/4,&description0[0]);
        matchGPU->SetDescriptors(1,keypoint1.size()/4,&description1[0]);

        int (*match_buf)[2] = new int[std::max(keypoint0.size()/4,keypoint1.size()/4)][2];
        //use the default thresholds. Check the declaration in SiftGPU.h
        int num_match = matchGPU->GetSiftMatch(std::max(keypoint0.size()/4,keypoint1.size()/4), match_buf);
        std::cout << num_match << " sift matches were found;\n";

    }
    return 0;

}