 /*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: sac_model_line.h 20633 2009-08-04 07:19:09Z tfoote $
 *
 */

/** \author Radu Bogdan Rusu */

#ifndef _SAMPLE_CONSENSUS_SACMODELLINE_H_
#define _SAMPLE_CONSENSUS_SACMODELLINE_H_

#include <sac_model.h>
#include <model_types.h>

/** \brief Define the maximum number of iterations for selecting 2 unique points */
#define MAX_ITERATIONS_UNIQUE 1000

namespace sample_consensus
{
  /** \brief A Sample Consensus Model class for 3D line segmentation.
    */
  class SACModelLine : public SACModel
  {
    public:
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Constructor for base SACModelLine. */
      SACModelLine () { }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Destructor for base SACModelLine. */
      virtual ~SACModelLine () { }

      virtual void getSamples (int &iterations, std::vector<int> &samples);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Test whether the given model coefficients are valid given the input point cloud data.
        * \param model_coefficients the model coefficients that need to be tested
        * \todo implement this
        */
      bool testModelCoefficients (const std::vector<double> &model_coefficients) { return true; }

      virtual bool computeModelCoefficients (const std::vector<int> &samples);

      virtual void refitModel (const std::vector<int> &inliers, std::vector<double> &refit_coefficients);
      virtual void getDistancesToModel (const std::vector<double> &model_coefficients, std::vector<double> &distances);
      virtual void selectWithinDistance (const std::vector<double> &model_coefficients, double threshold, std::vector<int> &inliers);

      virtual void projectPoints (const std::vector<int> &inliers, const std::vector<double> &model_coefficients, PointCloud &projected_points);

      virtual void projectPointsInPlace (const std::vector<int> &inliers, const std::vector<double> &model_coefficients);
      virtual bool doSamplesVerifyModel (const std::set<int> &indices, double threshold);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Return an unique id for this model (SACMODEL_LINE). */
      virtual int getModelType () { return (SACMODEL_LINE); }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Compute the cross product between two points (vectors).
        * \param p1 the first point/vector
        * \param p2 the second point/vector
        */
      inline pcl::PointXYZ
        cross (const pcl::PointXYZ &p1, const pcl::PointXYZ &p2)
      {
        pcl::PointXYZ r;
        r.x = p1.y * p2.z - p1.z * p2.y;
        r.y = p1.z * p2.x - p1.x * p2.z;
        r.z = p1.x * p2.y - p1.y * p2.x;
        return (r);
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Compute the centroid of a set of points using their indices and return it as a Point32 message.
        * \param points the input point cloud
        * \param indices the point cloud indices that need to be used
        * \param centroid the output centroid
        */
      inline void
        computeCentroid (const PointCloud &points, const std::vector<int> &indices, pcl::PointXYZ &centroid)
      {
        centroid.x = centroid.y = centroid.z = 0;
        // For each point in the cloud
        for (unsigned int i = 0; i < indices.size (); i++)
        {
          centroid.x += points.points.at (indices.at (i)).x;
          centroid.y += points.points.at (indices.at (i)).y;
          centroid.z += points.points.at (indices.at (i)).z;
        }

        centroid.x /= indices.size ();
        centroid.y /= indices.size ();
        centroid.z /= indices.size ();
      }
      
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Compute the 3x3 covariance matrix of a given set of points using their indices.
        * The result is returned as a Eigen::Matrix3d.
        * \note The (x-y-z) centroid is also returned as a Point32 message.
        * \param points the input point cloud
        * \param indices the point cloud indices that need to be used
        * \param covariance_matrix the 3x3 covariance matrix
        * \param centroid the computed centroid
        */
      inline void
        computeCovarianceMatrix (const PointCloud &points, const std::vector<int> &indices, Eigen::Matrix3d &covariance_matrix, pcl::PointXYZ &centroid)
      {
        computeCentroid (points, indices, centroid);

        // Initialize to 0
        covariance_matrix = Eigen::Matrix3d::Zero ();

        for (unsigned int j = 0; j < indices.size (); j++)
        {
          covariance_matrix (0, 0) += (points.points[indices.at (j)].x - centroid.x) * (points.points[indices.at (j)].x - centroid.x);
          covariance_matrix (0, 1) += (points.points[indices.at (j)].x - centroid.x) * (points.points[indices.at (j)].y - centroid.y);
          covariance_matrix (0, 2) += (points.points[indices.at (j)].x - centroid.x) * (points.points[indices.at (j)].z - centroid.z);

          covariance_matrix (1, 0) += (points.points[indices.at (j)].y - centroid.y) * (points.points[indices.at (j)].x - centroid.x);
          covariance_matrix (1, 1) += (points.points[indices.at (j)].y - centroid.y) * (points.points[indices.at (j)].y - centroid.y);
          covariance_matrix (1, 2) += (points.points[indices.at (j)].y - centroid.y) * (points.points[indices.at (j)].z - centroid.z);

          covariance_matrix (2, 0) += (points.points[indices.at (j)].z - centroid.z) * (points.points[indices.at (j)].x - centroid.x);
          covariance_matrix (2, 1) += (points.points[indices.at (j)].z - centroid.z) * (points.points[indices.at (j)].y - centroid.y);
          covariance_matrix (2, 2) += (points.points[indices.at (j)].z - centroid.z) * (points.points[indices.at (j)].z - centroid.z);
        }
      }
    };
}

#endif
