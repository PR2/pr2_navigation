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
 * $Id$
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b semantic_point_annotator annotates 3D point clouds with semantic labels.
(obsolete - should be removed soon.)

 **/

// ROS core
#include <ros/node.h>
// ROS messages
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Polygon.h>
#include <mapping_msgs/PolygonalMap.h>

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/msac.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>

#include <tf/transform_listener.h>
#include <angles/angles.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/distances.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/statistics.h>

#include <sys/time.h>

using namespace std;	

class SemanticPointAnnotator
{
  protected:
    ros::NodeHandle& node_;

  public:

    // ROS messages
    PointCloud cloud_, cloud_annotated_;
    Point32 z_axis_;

    ros::Publisher cloud_publisher_;
    ros::Subscriber cloud_normal_subscriber_;

    tf::TransformListener tf_;

    // Parameters
    int sac_min_points_per_model_, sac_min_points_left_;
    double sac_distance_threshold_, eps_angle_;

    double rule_floor_, rule_ceiling_, rule_wall_;
    double rule_table_min_, rule_table_max_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    SemanticPointAnnotator (ros::NodeHandle& anode) : node_ (anode)
    {
      node_.param ("rule_floor", rule_floor_, 0.1);           // Rule for FLOOR
      node_.param ("rule_ceiling", rule_ceiling_, 2.0);       // Rule for CEILING
      node_.param ("rule_table_min", rule_table_min_, 0.5);   // Rule for MIN TABLE
      node_.param ("rule_table_max", rule_table_max_, 1.5);   // Rule for MIN TABLE
      node_.param ("rule_wall", rule_wall_, 2.0);             // Rule for WALL

      node_.param ("p_sac_min_points_left", sac_min_points_left_, 500);
      node_.param ("p_sac_min_points_per_model", sac_min_points_per_model_, 100);  // 100 points at high resolution
      node_.param ("p_sac_distance_threshold", sac_distance_threshold_, 0.03);     // 3 cm
      node_.param ("p_eps_angle_", eps_angle_, 10.0);                              // 10 degrees
      eps_angle_ = angles::from_degrees (eps_angle_);                               // convert to radians

      cloud_normal_subscriber_ = node_.subscribe ("cloud_normals",1,&SemanticPointAnnotator::cloud_cb,this);//cloud_,
      cloud_publisher_ = node_.advertise<PointCloud> ("cloud_annotated", 1);

      z_axis_.x = 0; z_axis_.y = 0; z_axis_.z = 1;

      cloud_annotated_.channels.resize (3);
      //cloud_annotated_.channels[0].name = "intensities";
      cloud_annotated_.channels[0].name = "r";
      cloud_annotated_.channels[1].name = "g";
      cloud_annotated_.channels[2].name = "b";
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      fitSACPlane (PointCloud *points, vector<int> *indices, vector<vector<int> > &inliers, vector<vector<double> > &coeff)
    {
      if ((int)indices->size () < sac_min_points_per_model_)
        return;

      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
      sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, sac_distance_threshold_);
      sac->setMaxIterations (100);
      sac->setProbability (0.95);
      model->setDataSet (points, *indices);

      PointCloud pts (*points);
      int nr_points_left = indices->size ();
      while (nr_points_left > sac_min_points_left_)
      {
        // Search for the best plane
        if (sac->computeModel ())
        {
          // Obtain the inliers and the planar model coefficients
          if ((int)sac->getInliers ().size () < sac_min_points_per_model_)
            break;
          inliers.push_back (sac->getInliers ());
          coeff.push_back (sac->computeCoefficients ());
          fprintf (stderr, "> Found a model supported by %d inliers: [%g, %g, %g, %g]\n", sac->getInliers ().size (),
                   coeff[coeff.size () - 1][0], coeff[coeff.size () - 1][1], coeff[coeff.size () - 1][2], coeff[coeff.size () - 1][3]);

          // Project the inliers onto the model
          //model->projectPointsInPlace (sac->getInliers (), coeff[coeff.size () - 1]);

          // Remove the current inliers in the model
          nr_points_left = sac->removeInliers ();
        }
      }
    }

//    void pathCallback(const manipulation_msgs::JointTrajConstPtr &path_msg);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb (const PointCloudConstPtr &cloud_in)
    {
      cloud_ = *cloud_in;
      PointStamped base_link_origin, map_origin;
      base_link_origin.point.x = base_link_origin.point.y = base_link_origin.point.z = 0.0;
      base_link_origin.header.frame_id = "base_link";

      tf_.transformPoint ("base_link", base_link_origin, map_origin);

      ROS_INFO ("Received %d data points. Current robot pose is %g, %g, %g", cloud_.points.size (), map_origin.point.x, map_origin.point.y, map_origin.point.z);

      cloud_annotated_.header = cloud_.header;

      int nx = cloud_geometry::getChannelIndex (&cloud_, "nx");
      int ny = cloud_geometry::getChannelIndex (&cloud_, "ny");
      int nz = cloud_geometry::getChannelIndex (&cloud_, "nz");

      if ( (cloud_.channels.size () < 3) || (nx == -1) || (ny == -1) || (nz == -1) )
      {
        ROS_ERROR ("This PointCloud message does not contain normal information!");
        return;
      }

      timeval t1, t2;
      gettimeofday (&t1, NULL);

      // Select points whose normals are parallel with the Z-axis
      vector<int> z_axis_indices, xy_axis_indices;
      cloud_geometry::getPointIndicesAxisParallelNormals (&cloud_, nx, ny, nz, eps_angle_, &z_axis_, z_axis_indices);
      cloud_geometry::getPointIndicesAxisPerpendicularNormals (&cloud_, nx, ny, nz, eps_angle_, &z_axis_, xy_axis_indices);

      // Find the dominant planes
      vector<vector<int> > inliers_parallel, inliers_perpendicular;
      vector<vector<double> > coeff;
      // Find all planes parallel with XY
      fitSACPlane (&cloud_, &z_axis_indices, inliers_parallel, coeff);
      // Find all planes perpendicular to XY
      fitSACPlane (&cloud_, &xy_axis_indices, inliers_perpendicular, coeff);

      // Mark points in the output cloud
      int total_p = 0, nr_p = 0;
      for (unsigned int i = 0; i < inliers_parallel.size (); i++)
        total_p += inliers_parallel[i].size ();
      for (unsigned int i = 0; i < inliers_perpendicular.size (); i++)
        total_p += inliers_perpendicular[i].size ();

      cloud_annotated_.points.resize (total_p);
      cloud_annotated_.channels[0].values.resize (total_p);
      cloud_annotated_.channels[1].values.resize (total_p);
      cloud_annotated_.channels[2].values.resize (total_p);

      // Get all planes parallel to the floor (perpendicular to Z)
      Point32 robot_origin;
      robot_origin.x = map_origin.point.x;
      robot_origin.y = map_origin.point.y;
      robot_origin.z = map_origin.point.z;
      for (unsigned int i = 0; i < inliers_parallel.size (); i++)
      {
        // Compute a distance from 0,0,0 to the plane
        double distance = cloud_geometry::distances::pointToPlaneDistance (&robot_origin, coeff[i]);

        double r = 1.0, g = 1.0, b = 1.0;
        // Test for floor
        if (distance < rule_floor_)
        {
          r = 0.6; g = 0.67; b = 0.01;
        }
        // Test for ceiling
        if (distance > rule_ceiling_)
        {
          r = 0.8; g = 0.63; b = 0.33;
        }
        // Test for tables
        if (distance > rule_table_min_ && distance < rule_table_max_)
        {
          r = 0.0; g = 1.0; b = 0.0;
        }

        for (unsigned int j = 0; j < inliers_parallel[i].size (); j++)
        {
          cloud_annotated_.points[nr_p].x = cloud_.points.at (inliers_parallel[i].at (j)).x;
          cloud_annotated_.points[nr_p].y = cloud_.points.at (inliers_parallel[i].at (j)).y;
          cloud_annotated_.points[nr_p].z = cloud_.points.at (inliers_parallel[i].at (j)).z;
          //cloud_annotated_.channels[0].values[i] = intensity_value;
          cloud_annotated_.channels[0].values[nr_p] = r;
          cloud_annotated_.channels[1].values[nr_p] = g;
          cloud_annotated_.channels[2].values[nr_p] = b;
          nr_p++;
        }
      }

      // Get all planes perpendicular to the floor (parallel to XY)
      for (unsigned int i = 0; i < inliers_perpendicular.size (); i++)
      {
        // Get the minimum and maximum bounds of the plane
        Point32 minP, maxP;
        cloud_geometry::statistics::getMinMax (&cloud_, &inliers_perpendicular[i], minP, maxP);

        double r, g, b;
        r = g = b = 1.0;

        // Test for wall
        if (maxP.z > rule_wall_)
        {
          r = rand () / (RAND_MAX + 1.0);
          g = rand () / (RAND_MAX + 1.0);
          b = rand () / (RAND_MAX + 1.0);
          r = r * .3;
          b = b * .3 + .7;
          g = g * .3;
        }
        for (unsigned int j = 0; j < inliers_perpendicular[i].size (); j++)
        {
          cloud_annotated_.points[nr_p].x = cloud_.points.at (inliers_perpendicular[i].at (j)).x;
          cloud_annotated_.points[nr_p].y = cloud_.points.at (inliers_perpendicular[i].at (j)).y;
          cloud_annotated_.points[nr_p].z = cloud_.points.at (inliers_perpendicular[i].at (j)).z;
          //cloud_annotated_.channels[0].values[i] = intensity_value;
          cloud_annotated_.channels[0].values[nr_p] = r;
          cloud_annotated_.channels[1].values[nr_p] = g;
          cloud_annotated_.channels[2].values[nr_p] = b;
          nr_p++;
        }
      }

      gettimeofday (&t2, NULL);
      double time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Number of points with normals approximately parallel to the Z axis: %d (%g seconds).", z_axis_indices.size (), time_spent);

      cloud_publisher_.publish (cloud_annotated_);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "semantic_point_annotator");
  ros::NodeHandle ros_node("~");
  SemanticPointAnnotator p (ros_node);
  ros::spin();

  return (0);
}
/* ]--- */

