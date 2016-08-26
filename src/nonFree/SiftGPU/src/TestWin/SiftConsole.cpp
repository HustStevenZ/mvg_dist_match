//
// Created by Sanqian on 16/7/25.
//

#include "stdlib.h"
#include <iostream>
using std::iostream;
#include "SiftConsole.h"
#include <SiftGPU.h>
#include <SiftFileLoader.h>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/OpenGL.h>
#include <GLUT/glut.h>
#endif

#ifdef __linux__

#include <GL/gl.h>
#include <GL/glx.h>
#include <GL/glut.h>
#endif


int main(int argc,char** argv)
{

    SiftConsole* sift = new SiftConsole(argc,argv);
    glutInit(&argc,argv);
    int id;
    glutInitDisplayMode (GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize (0,0);
    glutInitWindowPosition(0,0);
    id = glutCreateWindow ("SIFT_GPU");
//    CGLContextObj context;
//
//    CGLPixelFormatAttribute attributes[4] = {
//            kCGLPFAAccelerated,   // no software rendering
//            kCGLPFAOpenGLProfile, // core profile with the version stated below
//            (CGLPixelFormatAttribute) kCGLOGLPVersion_GL4_Core,
//            (CGLPixelFormatAttribute) 0
//    };
//
//    CGLPixelFormatObj pix;
//    CGLError errorCode;
//    GLint num; // stores the number of possible pixel formats
//    errorCode = CGLChoosePixelFormat( attributes, &pix, &num );
//    // add error checking here
//    errorCode = CGLCreateContext( pix, NULL, &context ); // second parameter can be another context for object sharing
//    // add error checking here
//    CGLDestroyPixelFormat( pix );
//
//    errorCode = CGLSetCurrentContext( context );

    char* extensions = (char*)glGetString(GL_EXTENSIONS);
    sift->RunSiftGPU(argc,argv);
    delete sift;
    glutDestroyWindow(id);
    return 0;
}

SiftConsole::SiftConsole(int argc, char **argv) {

    _sift = new SiftGPUEX();



}
SiftConsole::~SiftConsole() {
    delete _sift;
}
//
//void SiftConsole::ParseSiftParam(int argc, char **argv) {
//    _sift->ParseParam(argc, argv);
//    _sift->SetVerbose(5);
//}

void SiftConsole::RunSiftGPU(int argc, char **argv) {

    _sift->ParseParam(argc, argv);
//    _sift->CreateContextGL();
    if(_sift->RunSIFT())
    {
        _sift->SetVerbose(2);

    }else
    {
        exit(0);
    }
}
