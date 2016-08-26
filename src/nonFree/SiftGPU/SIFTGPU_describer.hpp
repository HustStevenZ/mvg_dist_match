//
// Created by steven on 16-7-28.
//

#ifndef OPENMVG_SIFTGPU_DESCRIBER_HPP_H
#define OPENMVG_SIFTGPU_DESCRIBER_HPP_H

#include <cereal/cereal.hpp>
#include <iostream>
#include <numeric>
#include <vector>
#ifdef __linux__
#include <GL/glut.h>

#endif
#ifdef __APPLE__

#include <OpenGL/gl.h>
#include <GLUT/glut.h>

#endif

extern "C" {
#include "nonFree/SiftGPU/src/SiftGPU/SiftGPU.h"
}

namespace openMVG {
    namespace features {


        typedef enum _SIFTGPU_ENGINE{
            SIFTGPU_ENGINE_GLSL=0,
            SIFTGPU_ENGINE_CUDA=1,
            SIFTGPU_ENGINE_CL=2
        }SIFTGPU_ENGINE;
        inline void siftGPUDescToUChar(
                float descr[128],
                Descriptor<unsigned char,128> & descriptor,
                bool brootSift = false)
        {
            if (brootSift)  {
                // rootsift = sqrt( sift / sum(sift) );
                const float sum = accumulate(descr, descr+128, 0.0f);
                for (int k=0;k<128;++k)
                    descriptor[k] = static_cast<unsigned char>(512.f*sqrt(descr[k]/sum));
            }
            else
                for (int k=0;k<128;++k)
                    descriptor[k] = static_cast<unsigned char>(512.f*descr[k]);
        }


        class SIFTGPU_Image_describer : public Image_describer
        {
        public:
            SIFTGPU_Image_describer( bool bOrientation = true,SIFTGPU_ENGINE engine= SIFTGPU_ENGINE_GLSL)
                    :Image_describer(), _bOrientation(bOrientation),_engine(engine)
            {
//                vl_constructor();



//                siftgpu->_edge_threshold=10.0f;
//                siftgpu->_dog_threshold = 0.04f;
                siftgpu = new SiftGPUEX();
//
//                siftgpu->CreateContextGL();
//                siftgpu->VerifyContextGL();
                siftgpu->SetVerbose(-2);
                int argc = 2;
                char* argv[2];
                if(_engine == SIFTGPU_ENGINE_CL)
                {
                    argv[0] = (char*)malloc(sizeof(char)*(strlen("hello")+1));
                    argv[1] = (char*)malloc(sizeof(char)*(strlen("-cl")+1));
                    strcpy(argv[0],"hello");
                    argv[0][strlen("hello")] = '\0';
                    strcpy(argv[1],"-cl");
                    argv[1][strlen("-cl")] = '\0';
//                    char* argv[]={"hello","-cl"};
                }
                else if(_engine == SIFTGPU_ENGINE_CUDA)
                {

                    argv[0] = (char*)malloc(sizeof(char)*(strlen("hello")+1));
                    argv[1] = (char*)malloc(sizeof(char)*(strlen("-cuda")+1));
                    strcpy(argv[0],"hello");
                    argv[0][strlen("hello")] = '\0';
                    strcpy(argv[1],"-cuda");
                    argv[1][strlen("-cuda")] = '\0';
//                    char* argv[]={"hello","-cuda"};
                }
                else
                {
                    argv[0] = (char*)malloc(sizeof(char)*(strlen("hello")+1));
                    argv[1] = (char*)malloc(sizeof(char)*(strlen("-glsl")+1));
                    strcpy(argv[0],"hello");
                    argv[0][strlen("hello")] = '\0';
                    strcpy(argv[1],"-glsl");
                    argv[1][strlen("-glsl")] = '\0';
//                    char* argv[]={"hello","-glsl"};
                }
                glutInit(&argc,argv);
                int id;
                glutInitDisplayMode (GLUT_RGBA | GLUT_DOUBLE);
                glutInitWindowSize (0,0);
                glutInitWindowPosition(0,0);
                id = glutCreateWindow ("SIFT_GPU");

////                char[]
                siftgpu->ParseParam(argc,argv);
            }

            ~SIFTGPU_Image_describer()
            {
                delete siftgpu;
//                vl_destructor();
            }


            bool Set_configuration_preset(EDESCRIBER_PRESET preset)
            {
//                switch(preset)
//                {
//                    case NORMAL_PRESET:
//                        _params._peak_threshold = 0.04f;
//                        break;
//                    case HIGH_PRESET:
//                        _params._peak_threshold = 0.01f;
//                        break;
//                    case ULTRA_PRESET:
//                        _params._peak_threshold = 0.01f;
//                        _params._first_octave = -1;
//                        break;
//                    default:
//                        return false;
//                }
                return true;
            }
            /**
            @brief Detect regions on the image and compute their attributes (description)
            @param image Image.
            @param regions The detected regions and attributes (the caller must delete the allocated data)
            @param mask 8-bit gray image for keypoint filtering (optional).
               Non-zero values depict the region of interest.
            */
            bool Describe(const image::Image<unsigned char>& image,
                          std::unique_ptr<Regions> &regions,
                          const image::Image<unsigned char> * mask = NULL)
            {
//                  siftgpu->SetVerbose(4);
                const int w = image.Width(), h = image.Height();
                //Convert to float
                const image::Image<float> If(image.GetMat().cast<float>());
//
//                VlSiftFilt *filt = vl_sift_new(w, h,
//                                               _params._num_octaves, _params._num_scales, _params._first_octave);
//                if (_params._edge_threshold >= 0)
//                    vl_sift_set_edge_thresh(filt, _params._edge_threshold);
//                if (_params._peak_threshold >= 0)
//                    vl_sift_set_peak_thresh(filt, 255*_params._peak_threshold/_params._num_scales);

                Descriptor<vl_sift_pix, 128> descr;
                Descriptor<unsigned char, 128> descriptor;
//
//                // Process SIFT computation
//                vl_sift_process_first_octave(filt, If.data());

                Allocate(regions);

                // Build alias to cached data
                SIFT_Regions * regionsCasted = dynamic_cast<SIFT_Regions*>(regions.get());
                // reserve some memory for faster keypoint saving
                regionsCasted->Features().reserve(2000);
                regionsCasted->Descriptors().reserve(2000);
                std::cout<<"Get Feature from image "<<image.imageFilePath<<std::endl;
                if(siftgpu->RunSIFT(image.Width(),image.Height(),image.data(),GL_LUMINANCE,GL_BYTE))
//                if(siftgpu->RunSIFT(image.imageFilePath.c_str()));
                {
                    vector<SiftGPU::SiftKeypoint> key;
                    vector<float> description;
                    key.resize(siftgpu->GetFeatureNum());
                    std::cout<<"Found "<<siftgpu->GetFeatureNum()<<" features"<<std::endl;
                    description.resize(siftgpu->GetFeatureNum()*128);
                    siftgpu->GetFeatureVector(&key[0],&description[0]);

                    for(int i = 0;i<siftgpu->GetFeatureNum();i++)
                    {
                        const SIOPointFeature fp(key[i].x, key[i].y,
                                                 key[i].s, key[i].o);
                        siftDescToUChar(&description[i*128], descriptor);
                        regionsCasted->Descriptors().push_back(descriptor);
                        regionsCasted->Features().push_back(fp);
                    }
                }
//                while (true) {
//                    vl_sift_detect(filt);

//                    VlSiftKeypoint const *keys  = vl_sift_get_keypoints(filt);
//                    const int nkeys = vl_sift_get_nkeypoints(filt);

                    // Update gradient before launching parallel extraction
//                    vl_sift_update_gradient(filt);
//
//#ifdef OPENMVG_USE_OPENMP
//#pragma omp parallel for private(descr, descriptor)
//#endif
//                    for (int i = 0; i < nkeys; ++i) {
//
//                        // Feature masking
//                        if (mask)
//                        {
//                            const image::Image<unsigned char> & maskIma = *mask;
//                            if (maskIma(keys[i].y, keys[i].x) == 0)
//                                continue;
//                        }
//
//                        double angles [4] = {0.0, 0.0, 0.0, 0.0};
//                        int nangles = 1; // by default (1 upright feature)
//                        if (_bOrientation)
//                        { // compute from 1 to 4 orientations
////                            nangles = vl_sift_calc_keypoint_orientations(filt, angles, keys+i);
//                        }
//
//                        for (int q=0 ; q < nangles ; ++q) {
////                            vl_sift_calc_keypoint_descriptor(filt, &descr[0], keys+i, angles[q]);
//                            const SIOPointFeature fp(keys[i].x, keys[i].y,
//                                                     keys[i].sigma, static_cast<float>(angles[q]));
//
//                            siftDescToUChar(&descr[0], descriptor, _params._root_sift);
//#ifdef OPENMVG_USE_OPENMP
//#pragma omp critical
//#endif
//                            {
//                                regionsCasted->Descriptors().push_back(descriptor);
//                                regionsCasted->Features().push_back(fp);
//                            }
//                        }
//                    }
////                    if (vl_sift_process_next_octave(filt))
//                        break; // Last octave
//                }
//                vl_sift_delete(filt);

                return true;
            };

            /// Allocate Regions type depending of the Image_describer
            void Allocate(std::unique_ptr<Regions> &regions) const
            {
                regions.reset( new SIFT_Regions );
            }

            template<class Archive>
            void serialize( Archive & ar )
            {
                ar(
                        cereal::make_nvp("params", _params),
                        cereal::make_nvp("bOrientation", _bOrientation));
            }

        private:
            SiftParams _params;
            bool _bOrientation;
            SiftGPU* siftgpu = nullptr;
            SIFTGPU_ENGINE _engine= SIFTGPU_ENGINE_GLSL;
        };

    } // namespace features
} // namespace openMVG

#include <cereal/types/polymorphic.hpp>
#include <cereal/archives/json.hpp>
CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::features::SIFTGPU_Image_describer, "SIFTGPU_Image_describer");

#endif // OPENMVG_SIFTGPU_DESCRIBER_HPP_H


