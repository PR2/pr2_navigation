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

\author Radu Bogdan Rusu, Eitan Marder-Eppstein

@b sac_inc_ground_removal returns a cloud with the ground plane extracted.

 **/

// ROS core
#include <ros/ros.h>
// ROS messages
#include <Eigen/QR>
#include <Eigen/Eigenvalues>
// Sample Consensus
#include <sac.h>
#include <ransac.h>
#include <sac_model_line.h>

#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#include <sys/time.h>

#include <boost/thread.hpp>

#include <pcl_ros/transforms.h>

using namespace std;

class IncGroundRemoval
{
  protected:
    ros::NodeHandle& node_;

  public:

    // ROS messages
    sample_consensus::PointCloud laser_cloud_, cloud_, cloud_noground_;

    tf::TransformListener tf_;
    geometry_msgs::PointStamped viewpoint_cloud_;
    tf::MessageFilter<sensor_msgs::PointCloud2>* cloud_notifier_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>* cloud_subscriber_;
  
    // Parameters
    double z_threshold_, ground_slope_threshold_;
    int sac_min_points_per_model_, sac_max_iterations_;
    double sac_distance_threshold_;
    double sac_fitting_distance_threshold_;
    int planar_refine_;
    std::string robot_footprint_frame_, laser_tilt_mount_frame_;

    ros::Publisher cloud_publisher_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    IncGroundRemoval (ros::NodeHandle& anode) : node_ (anode)
    {
      node_.param ("z_threshold", z_threshold_, 0.1);                          // 10cm threshold for ground removal
      node_.param ("ground_slope_threshold", ground_slope_threshold_, 0.0);                          // 0% slope threshold for ground removal
      node_.param ("sac_distance_threshold", sac_distance_threshold_, 0.03);   // 3 cm threshold
      node_.param ("sac_fitting_distance_threshold", sac_fitting_distance_threshold_, 0.015);   // 1.5 cm threshold

      node_.param ("planar_refine", planar_refine_, 1);                        // enable a final planar refinement step?
      node_.param ("sac_min_points_per_model", sac_min_points_per_model_, 6);  // 6 points minimum per line
      node_.param ("sac_max_iterations", sac_max_iterations_, 200);            // maximum 200 iterations
      node_.param ("robot_footprint_frame", robot_footprint_frame_, std::string("base_footprint"));
      node_.param ("laser_tilt_mount_frame", laser_tilt_mount_frame_, std::string("laser_tilt_mount_link"));

      string cloud_topic ("tilt_laser_cloud_filtered");

      bool topic_found = false;
      std::vector<ros::master::TopicInfo> t_list;
      ros::master::getTopics (t_list);
      for (vector<ros::master::TopicInfo>::iterator it = t_list.begin (); it != t_list.end (); it++)
      {
        if (it->name == cloud_topic)
        {
          topic_found = true;
          break;
        }
      }
      if (!topic_found)
        ROS_WARN ("Trying to subscribe to %s, but the topic doesn't exist!", cloud_topic.c_str ());

      ros::NodeHandle public_node;

      //subscribe (cloud_topic.c_str (), laser_cloud_, &IncGroundRemoval::cloud_cb, 1);
      cloud_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(public_node,cloud_topic,50);
      cloud_notifier_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(*cloud_subscriber_,tf_,"odom_combined",50);
      cloud_notifier_->registerCallback(boost::bind(&IncGroundRemoval::cloud_cb,this,_1));

//      cloud_notifier_ = new tf::MessageNotifier<sample_consensus::PointCloud> (&tf_, node_,
//                        boost::bind (&IncGroundRemoval::cloud_cb, this, _1), cloud_topic, "odom_combined", 50);

      cloud_publisher_ = public_node.advertise<sensor_msgs::PointCloud2> ("cloud_ground_filtered", 1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~IncGroundRemoval () { delete cloud_notifier_; }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      updateParametersFromServer ()
    {
      node_.getParam ("z_threshold", z_threshold_);
      node_.getParam ("ground_slope_threshold", ground_slope_threshold_);
      node_.getParam ("sac_fitting_distance_threshold", sac_fitting_distance_threshold_);
      node_.getParam ("sac_distance_threshold", sac_distance_threshold_);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Find a line model in a point cloud given via a set of point indices with SAmple Consensus methods
      * \param points the point cloud message
      * \param indices a pointer to a set of point cloud indices to test
      * \param inliers the resultant inliers
      */
    bool
      fitSACLine (sample_consensus::PointCloud *points, vector<int> *indices, vector<int> &inliers)
    {
      if ((int)indices->size () < sac_min_points_per_model_)
        return (false);

      // Create and initialize the SAC model
      sample_consensus::SACModelLine *model = new sample_consensus::SACModelLine ();
      sample_consensus::SAC *sac            = new sample_consensus::RANSAC (model, sac_fitting_distance_threshold_);
      sac->setMaxIterations (sac_max_iterations_);
      sac->setProbability (0.99);

      model->setDataSet (points, *indices);

      vector<double> line_coeff;
      // Search for the best model
      if (sac->computeModel (0))
      {
        // Obtain the inliers and the planar model coefficients
        if ((int)sac->getInliers ().size () < sac_min_points_per_model_)
          return (false);
        //inliers    = sac->getInliers ();

        sac->computeCoefficients (line_coeff);             // Compute the model coefficients
        sac->refineCoefficients (line_coeff);              // Refine them using least-squares
        model->selectWithinDistance (line_coeff, sac_distance_threshold_, inliers);

        // Project the inliers onto the model
        //model->projectPointsInPlace (sac->getInliers (), coeff);
      }
      else
        return (false);

      delete sac;
      delete model;
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Flip (in place) the estimated normal of a point towards a given viewpoint
      * \param normal the plane normal to be flipped
      * \param point a given point
      * \param viewpoint the viewpoint
      */
    inline void
      flipNormalTowardsViewpoint (Eigen::Vector4d &normal, const pcl::PointXYZ &point, const geometry_msgs::PointStamped &viewpoint)
    {
      // See if we need to flip any plane normals
      float vp_m[3];
      vp_m[0] = viewpoint.point.x - point.x;
      vp_m[1] = viewpoint.point.y - point.y;
      vp_m[2] = viewpoint.point.z - point.z;

      // Dot product between the (viewpoint - point) and the plane normal
      double cos_theta = (vp_m[0] * normal (0) + vp_m[1] * normal (1) + vp_m[2] * normal (2));

      // Flip the plane normal
      if (cos_theta < 0)
      {
        for (int d = 0; d < 3; d++)
          normal (d) *= -1;
        // Hessian form (D = nc . p_plane (centroid here) + p)
        normal (3) = -1 * (normal (0) * point.x + normal (1) * point.y + normal (2) * point.z);
      }
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the centroid of a set of points using their indices and return it as a Point32 message.
      * \param points the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param centroid the output centroid
      */
    inline void
      computeCentroid (const sample_consensus::PointCloud &points, const std::vector<int> &indices, pcl::PointXYZ &centroid)
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
      computeCovarianceMatrix (const sample_consensus::PointCloud &points, const std::vector<int> &indices, Eigen::Matrix3d &covariance_matrix, pcl::PointXYZ &centroid)
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

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the Least-Squares plane fit for a given set of points, using their indices,
      * and return the estimated plane parameters together with the surface curvature.
      * \param points the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param plane_parameters the plane parameters as: a, b, c, d (ax + by + cz + d = 0)
      * \param curvature the estimated surface curvature as a measure of
      * \f[
      * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
      * \f]
      */
    void
      computePointNormal (const sample_consensus::PointCloud &points, const std::vector<int> &indices, Eigen::Vector4d &plane_parameters, double &curvature)
    {
      pcl::PointXYZ centroid;
      // Compute the 3x3 covariance matrix
      Eigen::Matrix3d covariance_matrix;
      computeCovarianceMatrix (points, indices, covariance_matrix, centroid);

      // Extract the eigenvalues and eigenvectors
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ei_symm (covariance_matrix);
      Eigen::Vector3d eigen_values  = ei_symm.eigenvalues ();
      Eigen::Matrix3d eigen_vectors = ei_symm.eigenvectors ();

      // Normalize the surface normal (eigenvector corresponding to the smallest eigenvalue)
      // Note: Remember to take care of the eigen_vectors ordering
      double norm = sqrt ( eigen_vectors (0, 0) * eigen_vectors (0, 0) +
                           eigen_vectors (1, 0) * eigen_vectors (1, 0) +
                           eigen_vectors (2, 0) * eigen_vectors (2, 0));
      plane_parameters (0) = eigen_vectors (0, 0) / norm;
      plane_parameters (1) = eigen_vectors (1, 0) / norm;
      plane_parameters (2) = eigen_vectors (2, 0) / norm;

      // Hessian form (D = nc . p_plane (centroid here) + p)
      plane_parameters (3) = -1 * (plane_parameters (0) * centroid.x + plane_parameters (1) * centroid.y + plane_parameters (2) * centroid.z);

      // Compute the curvature surface change
      curvature = fabs ( eigen_values (0) / (eigen_values (0) + eigen_values (1) + eigen_values (2)) );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb (const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
      pcl::fromROSMsg(*msg,laser_cloud_);
      //check to see if the point cloud is empty
      if(laser_cloud_.points.empty()){
        ROS_DEBUG("Received an empty point cloud");
        cloud_publisher_.publish(msg);
        return;
      }

      try{
        pcl_ros::transformPointCloud("odom_combined", laser_cloud_, cloud_, tf_);
      }
      catch(tf::TransformException &ex){
        ROS_ERROR("Can't transform cloud for ground plane detection");
        return;
      }

      ROS_DEBUG ("Received %d data points.", (int)cloud_.points.size ());
      if (cloud_.points.size () == 0)
        return;

      //updateParametersFromServer ();
      // Copy the header
      cloud_noground_.header = cloud_.header;

      timeval t1, t2;
      gettimeofday (&t1, NULL);

      // Get the cloud viewpoint
      getCloudViewPoint (cloud_.header.frame_id, viewpoint_cloud_, &tf_);

      // Transform z_threshold_ from the parameter parameter frame (parameter_frame_) into the point cloud frame
      std_msgs::Header header = pcl_conversions::fromPCL(cloud_.header);
      double z_threshold_cloud = transformDoubleValueTF (z_threshold_, robot_footprint_frame_, cloud_.header.frame_id, header.stamp, &tf_);

      // Select points whose Z dimension is close to the ground (0,0,0 in base_footprint) or under a gentle slope (allowing for pitch/roll error)
      vector<int> possible_ground_indices (cloud_.points.size ());
      vector<int> all_indices (cloud_.points.size ());
      int nr_p = 0;
      for (unsigned int cp = 0; cp < cloud_.points.size (); cp++)
      {
        all_indices[cp] = cp;
        if (fabs (cloud_.points[cp].z) < z_threshold_cloud || // max height for ground
            cloud_.points[cp].z*cloud_.points[cp].z < ground_slope_threshold_ * (cloud_.points[cp].x*cloud_.points[cp].x + cloud_.points[cp].y*cloud_.points[cp].y)) // max slope for ground 
        {
          possible_ground_indices[nr_p] = cp;
          nr_p++;
        }
      }
      possible_ground_indices.resize (nr_p);

      ROS_DEBUG ("Number of possible ground indices: %d.", (int)possible_ground_indices.size ());

      vector<int> ground_inliers;

      // Find the dominant plane in the space of possible ground indices
      fitSACLine (&cloud_, &possible_ground_indices, ground_inliers);
      

      if (ground_inliers.size () == 0){
        ROS_DEBUG ("Couldn't fit a model to the scan.");
        //if we can't fit a line model to the scan, we have to assume all the possible ground inliers are on the ground
        ground_inliers = possible_ground_indices;
      }

      ROS_DEBUG ("Total number of ground inliers before refinement: %d.", (int)ground_inliers.size ());

      // Do we attempt to do a planar refinement to remove points "below" the plane model found ?
      if (planar_refine_ > 0)
      {
        // Get the remaining point indices
        vector<int> remaining_possible_ground_indices;
        sort (all_indices.begin (), all_indices.end ());
        sort (ground_inliers.begin (), ground_inliers.end ());
        set_difference (all_indices.begin (), all_indices.end (), ground_inliers.begin (), ground_inliers.end (),
                        inserter (remaining_possible_ground_indices, remaining_possible_ground_indices.begin ()));

        // Estimate the plane from the line inliers
        Eigen::Vector4d plane_parameters;
        double curvature;
        computePointNormal (cloud_, ground_inliers, plane_parameters, curvature);

        //make sure that there are inliers to refine
        if (!ground_inliers.empty ())
        {
          flipNormalTowardsViewpoint (plane_parameters, cloud_.points.at (ground_inliers[0]), viewpoint_cloud_);

          // Compute the distance from the remaining points to the model plane, and add to the inliers list if they are below
          for (unsigned int i = 0; i < remaining_possible_ground_indices.size (); i++)
          {
            double distance_to_ground  = pointToPlaneDistanceSigned (cloud_.points.at (remaining_possible_ground_indices[i]), plane_parameters);
            if (distance_to_ground >= 1e-6){
              continue;
            }
            ground_inliers.push_back (remaining_possible_ground_indices[i]);
          }
        }
      }
      ROS_DEBUG ("Total number of ground inliers after refinement: %d.", (int)ground_inliers.size ());

      // Get all the non-ground point indices
      vector<int> remaining_indices;
      sort (ground_inliers.begin (), ground_inliers.end ());
      sort (all_indices.begin(), all_indices.end());
      set_difference (all_indices.begin (), all_indices.end (), ground_inliers.begin (), ground_inliers.end (),
                      inserter (remaining_indices, remaining_indices.begin ()));

      // Prepare new arrays
      int nr_remaining_pts = remaining_indices.size ();
      cloud_noground_.points.resize (nr_remaining_pts);

      for (unsigned int i = 0; i < remaining_indices.size (); i++)
      {
        cloud_noground_.points[i].x = cloud_.points.at (remaining_indices[i]).x;
        cloud_noground_.points[i].y = cloud_.points.at (remaining_indices[i]).y;
        cloud_noground_.points[i].z = cloud_.points.at (remaining_indices[i]).z;
      }

      gettimeofday (&t2, NULL);
      double time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_DEBUG ("Number of points found on ground plane: %d ; remaining: %d (%g seconds).", (int)ground_inliers.size (),
                (int)remaining_indices.size (), time_spent);
      sensor_msgs::PointCloud2 out;
      pcl::toROSMsg(cloud_noground_, out);
      cloud_publisher_.publish (out);
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a point to a plane (signed) defined by ax+by+cz+d=0
      * \param p a point
      * \param plane_coefficients the normalized coefficients (a, b, c, d) of a plane
      */
    inline double
      pointToPlaneDistanceSigned (const pcl::PointXYZ &p, const Eigen::Vector4d &plane_coefficients)
    {
      return ( plane_coefficients (0) * p.x + plane_coefficients (1) * p.y + plane_coefficients (2) * p.z + plane_coefficients (3) );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the view point from where the scans were taken in the incoming sample_consensus::PointCloud message frame
      * \param cloud_frame the point cloud message TF frame
      * \param viewpoint_cloud the resultant view point in the incoming cloud frame
      * \param tf a pointer to a TransformListener object
      */
    void
      getCloudViewPoint (string cloud_frame, geometry_msgs::PointStamped &viewpoint_cloud, tf::TransformListener *tf)
    {
      // Figure out the viewpoint value in the point cloud frame
      geometry_msgs::PointStamped viewpoint_laser;
      viewpoint_laser.header.frame_id = laser_tilt_mount_frame_;
      // Set the viewpoint in the laser coordinate system to 0, 0, 0
      viewpoint_laser.point.x = viewpoint_laser.point.y = viewpoint_laser.point.z = 0.0;

      try
      {
        tf->transformPoint (cloud_frame, viewpoint_laser, viewpoint_cloud);
        ROS_DEBUG ("Cloud view point in frame %s is: %g, %g, %g.", cloud_frame.c_str (),
                  viewpoint_cloud.point.x, viewpoint_cloud.point.y, viewpoint_cloud.point.z);
      }
      catch (tf::ConnectivityException)
      {
        ROS_WARN ("Could not transform a point from frame %s to frame %s!", viewpoint_laser.header.frame_id.c_str (), cloud_frame.c_str ());
        // Default to 0.05, 0, 0.942768
        viewpoint_cloud.point.x = 0.05; viewpoint_cloud.point.y = 0.0; viewpoint_cloud.point.z = 0.942768;
      }
      catch(tf::TransformException &ex){
        ROS_ERROR("Can't transform viewpoint for ground plan detection");
        viewpoint_cloud.point.x = 0.05; viewpoint_cloud.point.y = 0.0; viewpoint_cloud.point.z = 0.942768;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Transform a given point from its current frame to a given target frame
      * \param tf a pointer to a TransformListener object
      * \param target_frame the target frame to transform the point into
      * \param stamped_in the input point
      * \param stamped_out the output point
      */
    inline void
      transformPoint (tf::TransformListener *tf, const std::string &target_frame,
                      const tf::Stamped< pcl::PointXYZ > &stamped_in, tf::Stamped< pcl::PointXYZ > &stamped_out)
    {
      tf::Stamped<tf::Point> tmp;
      tmp.stamp_ = stamped_in.stamp_;
      tmp.frame_id_ = stamped_in.frame_id_;
      tmp[0] = stamped_in.x;
      tmp[1] = stamped_in.y;
      tmp[2] = stamped_in.z;

      try{
        tf->transformPoint (target_frame, tmp, tmp);
      }
      catch(tf::TransformException &ex){
        ROS_ERROR("Can't transform cloud for ground plane detection");
        return;
      }

      stamped_out.stamp_ = tmp.stamp_;
      stamped_out.frame_id_ = tmp.frame_id_;
      stamped_out.x = tmp[0];
      stamped_out.y = tmp[1];
      stamped_out.z = tmp[2];
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Transform a value from a source frame to a target frame at a certain moment in time with TF
      * \param val the value to transform
      * \param src_frame the source frame to transform the value from
      * \param tgt_frame the target frame to transform the value into
      * \param stamp a given time stamp
      * \param tf a pointer to a TransformListener object
      */
    inline double
      transformDoubleValueTF (double val, std::string src_frame, std::string tgt_frame, ros::Time stamp, tf::TransformListener *tf)
    {
      pcl::PointXYZ temp;
      temp.x = temp.y = 0;
      temp.z = val;
      tf::Stamped<pcl::PointXYZ> temp_stamped (temp, stamp, src_frame);
      transformPoint (tf, tgt_frame, temp_stamped, temp_stamped);
      return (temp_stamped.z);
    }

};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "sac_ground_removal");

  ros::NodeHandle ros_node ("~");

  // For efficiency considerations please make sure the input sample_consensus::PointCloud is in a frame with Z point upwards
  IncGroundRemoval p (ros_node);
  ros::spin ();

  return (0);
}
/* ]--- */

