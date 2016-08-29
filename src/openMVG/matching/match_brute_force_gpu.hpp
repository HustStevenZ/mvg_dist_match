// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_MATCHING_ARRAYMATCHER_BRUTE_FORCE_GPU_H_
#define OPENMVG_MATCHING_ARRAYMATCHER_BRUTE_FORCE_GPU_H_

#include "openMVG/matching/matching_interface.hpp"
#include "openMVG/matching/metric.hpp"
#include "nonFree/SiftGPU/src/SiftGPU/SiftGPU.h"
#include <algorithm>
#include <iostream>
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

#include <memory>

namespace openMVG {
namespace matching  {

/// Implement ArrayMatcher as a FLANN KDtree matcher.
// http://www.cs.ubc.ca/~mariusm/index.php/FLANN/FLANN
// David G. Lowe and Marius Muja
//
// By default use squared L2 metric (flann::L2<Scalar>)
// sqrt is monotonic so for performance reason we do not compute it.

template < typename Scalar = float, typename Metric = L2_Simple<Scalar> >
class ArrayMatcher_Brute_Force_GPU : public ArrayMatcher<Scalar, Metric>
{
  public:
  typedef typename Metric::ResultType DistanceType;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> BaseMat;

    ArrayMatcher_Brute_Force_GPU() = default ;

  virtual ~ArrayMatcher_Brute_Force_GPU()  {
    _dataSetVec.clear();
//    _index.reset();
//    glutDestroyWindow(glutWindowId);
    _matcher.reset();
  }

  /**
   * Build the matching structure
   *
   * \param[in] dataset   Input data.
   * \param[in] nbRows    The number of component.
   * \param[in] dimension Length of the data contained in the each
   *  row of the dataset.
   *
   * \return True if success.
   */
  void initGL()
  {
//    int argc = 0;
//    char** argv = nullptr;
//    glutInit(&argc,argv);
//    int id;
//    glutInitDisplayMode (GLUT_RGBA | GLUT_DOUBLE);
//    glutInitWindowSize (0,0);
//    glutInitWindowPosition(0,0);
//    glutWindowId = glutCreateWindow ("SIFT_GPU");
  }
  bool Build( const Scalar * dataset, int nbRows, int dimension)  {

    if (nbRows > 0)
    {
      _dimension = dimension;
      _rows = nbRows;
      for(int i =0;i<nbRows;i++)
        for(int j = 0;j<dimension;j++)
        {
          _dataSetVec.push_back(static_cast<float>(*(dataset+i*dimension+j)));
        }
//      _datasetM.reset(new Eigen::Map<BaseMat>( (Scalar*)dataset, nbRows, dimension));
//      initGL();

      _matcher.reset(new SiftMatchGPU());
#ifdef __APPLE__
      _matcher->SetLanguage(SiftMatchGPU::SIFTMATCH_GLSL);
#endif
#ifdef __linux__
      _matcher->SetLanguage(SiftMatchGPU::SIFTMATCH_CUDA);
#endif
      _matcher->CreateContextGL();

      return true;
    }
    return false;
  }

  /**
   * Search the nearest Neighbor of the scalar array query.
   *
   * \param[in]   query     The query array
   * \param[out]  indice    The indice of array in the dataset that
   *  have been computed as the nearest array.
   * \param[out]  distance  The distance between the two arrays.
   *
   * \return True if success.
   */
  bool SearchNeighbour( const Scalar * query, int * indice, DistanceType * distance)
  {
    if(_matcher.get()!= nullptr)
    {

      std::cout<<"single search is not implemented yet"<<std::endl;

      return false;
    }
    return false;
  }


/**
   * Search the N nearest Neighbor of the scalar array query.
   *
   * \param[in]   query           The query array
   * \param[in]   nbQuery         The number of query rows
   * \param[out]  indices   The corresponding (query, neighbor) indices
   * \param[out]  pvec_distances  The distances between the matched arrays.
   * \param[out]  NN              The number of maximal neighbor that will be searched.
   *
   * \return True if success.
   */
  bool SearchNeighbours
  (
    const Scalar * query, int nbQuery,
    IndMatches * pvec_indices,
    std::vector<DistanceType> * pvec_distances,
    size_t NN
  )
  {
    if(_matcher.get()!= nullptr)
    {
      std::vector<float> description1;
      for(int i =0;i<nbQuery;i++)
        for(int j = 0;j<_dimension;j++)
        {
          description1.push_back(static_cast<float>(*(query+i*_dimension+j)));
        }
      _matcher->SetDescriptors(0,_rows,&_dataSetVec[0]);
      _matcher->SetDescriptors(1,nbQuery,&description1[0]);

      int (*match_buf)[2] = new int[std::max(_rows,nbQuery)][2];
      int num_match = _matcher->GetSiftMatch(std::max(_rows,nbQuery), match_buf);
      //Ignore result since we only need to do the performance test


      return true;
    }
    return false;
  }

  private :

//  std::unique_ptr<Eigen::Map<BaseMat>> _datasetM;

    std::vector<float> _dataSetVec;
    std::unique_ptr<SiftMatchGPU> _matcher;
    int glutWindowId;
    int _dimension;
    int _rows;
};

} // namespace matching
} // namespace openMVG

#endif // OPENMVG_MATCHING_ARRAYMATCHER_BRUTE_FORCE_GPU_H_
