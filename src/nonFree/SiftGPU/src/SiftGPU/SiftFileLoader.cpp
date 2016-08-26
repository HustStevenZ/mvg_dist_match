//
// Created by Sanqian on 16/7/26.
//

#include <fstream>
#include <iostream>
#include "SiftFileLoader.h"
#include <stdlib.h>
#include <stdio.h>
#include <cstring>

void SiftFileLoader::loadFile(std::string fileName, std::vector<float> &keyPointBuffer, std::vector<float> &descriptionBuffer,int &descp_per_point) {
    std::ifstream file(fileName);

    if(file.is_open())
    {
        int featureNumber = 0;
        int siftFileType = 0;
        int linenumber = 0;
        while (!file.eof())
        {
            std::string line;
            std::getline(file,line);


            if(file.eof())
                return;
            int numFloat = 0;
            float floats[20] = {0.0f};

            int startPos = 0;
            int floatEndPos = 0;
            int floatindex = 0;
//            std::cout<<line<<std::endl;
            char* linestr= (char*)malloc(line.length()+1);
            strcpy(linestr,line.data());
            char* token = strtok(linestr," ");
            do
            {
                floats[floatindex] = atof(token);
                floatindex++;
                token = strtok(NULL," ");
            }while(token!=NULL);
//
//            floats[floatindex] = atof(token);
//            std::cout<<linestr<<std::endl;
//            std::string linecopy(linestr);
//            do{
//                floatEndPos = linecopy.find(' ',startPos);
//
//                if(floatEndPos!=std::string::npos)
//                {
//                    floats[floatindex] = atof((linecopy.substr(startPos,floatEndPos-1).c_str()));
//                    startPos = floatEndPos+1;
//                    floatindex++;
//                }
//            }while(floatEndPos!=std::string::npos);
//
//            floats[floatindex] = std::atoi((linecopy.substr(startPos,linecopy.length()-1).c_str()));
            if(floatindex == 2 && linenumber == 0)
            {
                //First line
                featureNumber = floats[0];
                siftFileType = floats[1];
                descp_per_point = siftFileType;
                //First line?
            }
            else if(floatindex == 4)
            {
                //Point line
                float x = floats[0];
                float y = floats[1];
                float scale = floats[2];
                float rotation_order = floats[3];
                keyPointBuffer.push_back(x);
                keyPointBuffer.push_back(y);
                keyPointBuffer.push_back(scale);
                keyPointBuffer.push_back(rotation_order);

            }
            else
            {
                for(int i = 0;i<floatindex;i++)
                {
                    float value = (floats[i]+0.5)/512.0f;
                    descriptionBuffer.push_back(value);
                }
            }
//            std::cout<<line<<std::endl;

            linenumber++;
        }
    }

}