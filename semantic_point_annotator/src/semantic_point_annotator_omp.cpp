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

@b semantic_point_annotator_omp annotates 3D point clouds with semantic labels.

 **/

// ROS core
#include <ros/ros.h>
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

// Kd Tree
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/distances.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/statistics.h>

#include <sys/time.h>

using namespace std;
using namespace mapping_msgs;

struct Region
{
  char region_type;
  vector<int> indices;
};

class SemanticPointAnnotator
{
  protected:
    ros::NodeHandle& node_;

  public:

    // ROS messages
    sensor_msgs::PointCloud cloud_, cloud_annotated_;
    geometry_msgs::Point32 z_axis_;
    PolygonalMap pmap_;

    tf::TransformListener tf_;

    // Parameters
    int sac_min_points_per_model_, sac_min_points_left_;
    double sac_distance_threshold_, eps_angle_, region_angle_threshold_, boundary_angle_threshold_;

    double rule_floor_, rule_ceiling_, rule_wall_;
    double rule_table_min_, rule_table_max_;

    double region_growing_tolerance_;

    bool polygonal_map_, concave_;
    int min_cluster_pts_;

    ros::Publisher polygonal_map_publisher_, cloud_publisher_;
    ros::Subscriber cloud_subscriber_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    SemanticPointAnnotator (ros::NodeHandle& anode) : node_ (anode)
    {
      node_.param ("rule_floor", rule_floor_, 0.3);           // Rule for FLOOR
      node_.param ("rule_ceiling", rule_ceiling_, 2.0);       // Rule for CEILING
      node_.param ("rule_table_min", rule_table_min_, 0.5);   // Rule for MIN TABLE
      node_.param ("rule_table_max", rule_table_max_, 1.5);   // Rule for MIN TABLE
      node_.param ("rule_wall", rule_wall_, 2.0);             // Rule for WALL

      node_.param ("min_cluster_pts", min_cluster_pts_, 10);  // Display only clusters with more than 10 points

      node_.param ("region_growing_tolerance", region_growing_tolerance_, 0.25);  // 10 cm

      node_.param ("region_angle_threshold", region_angle_threshold_, 30.0);   // Difference between normals in degrees for cluster/region growing
      region_angle_threshold_ = angles::from_degrees (region_angle_threshold_); // convert to radians

      node_.param ("p_sac_min_points_left", sac_min_points_left_, 10);
      node_.param ("p_sac_min_points_per_model", sac_min_points_per_model_, 10);   // 50 points at high resolution

      // This should be set to whatever the leaf_width factor is in the downsampler
      node_.param ("p_sac_distance_threshold", sac_distance_threshold_, 0.05);     // 5 cm

      node_.param ("p_eps_angle", eps_angle_, 15.0);                               // 15 degrees

      node_.param ("create_polygonal_map", polygonal_map_, true);            // Create a polygonal map ?
      node_.param ("concave", concave_, false);                              // Create concave hulls by default
      node_.param ("boundary_angle_threshold", boundary_angle_threshold_, 120.0); // Boundary angle threshold

      if (polygonal_map_)
       polygonal_map_publisher_ = node_.advertise<PolygonalMap> ("semantic_polygonal_map", 1);

      eps_angle_ = angles::from_degrees (eps_angle_);                // convert to radians

      if (concave_)
        ROS_INFO ("Concave hulls enabled. Angle threshold set to %g.", boundary_angle_threshold_);

      boundary_angle_threshold_ = angles::from_degrees (boundary_angle_threshold_);  // convert to radians

      string cloud_topic ("cloud_normals");
      std::vector<ros::master::TopicInfo> t_list;
      bool topic_found = false;
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

      cloud_subscriber_ = node_.subscribe(cloud_topic, 1, &SemanticPointAnnotator::cloud_cb, this);
      cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud> ("cloud_annotated", 1);

      z_axis_.x = 0; z_axis_.y = 0; z_axis_.z = 1;

      cloud_annotated_.channels.resize (1);
      cloud_annotated_.channels[0].name = "rgb";
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Decompose a region of space into clusters based on the euclidean distance between points
      * \param points the point cloud message
      * \param indices a list of point indices
      * \param tolerance the spatial tolerance as a measure in the L2 Euclidean space
      * \param clusters the resultant clusters
      * \param nx_idx
      * \param ny_idx
      * \param nz_idx
      * \param min_pts_per_cluster minimum number of points that a cluster may contain (default = 1)
      */
    void
      findClusters (const sensor_msgs::PointCloud &points, const vector<int> &indices, double tolerance, vector<Region> &clusters,
                    int nx_idx, int ny_idx, int nz_idx,
                    unsigned int min_pts_per_cluster = 1)
    {
      // Create a tree for these points
      cloud_kdtree::KdTree* tree = new cloud_kdtree::KdTreeANN (points, indices);

      // Create a bool vector of processed point indices, and initialize it to false
      vector<bool> processed;
      processed.resize (indices.size (), false);

      vector<int> nn_indices;
      vector<float> nn_distances;
      // Process all points in the indices vector
      for (unsigned int i = 0; i < indices.size (); i++)
      {
        if (processed[i])
          continue;

        vector<int> seed_queue;
        int sq_idx = 0;
        seed_queue.push_back (i);

        double norm_a = sqrt (points.channels[nx_idx].values[indices.at (i)] * points.channels[nx_idx].values[indices.at (i)] +
                              points.channels[ny_idx].values[indices.at (i)] * points.channels[ny_idx].values[indices.at (i)] +
                              points.channels[nz_idx].values[indices.at (i)] * points.channels[nz_idx].values[indices.at (i)]);

        processed[i] = true;

        while (sq_idx < (int)seed_queue.size ())
        {
          tree->radiusSearch (seed_queue.at (sq_idx), tolerance, nn_indices, nn_distances);

          for (unsigned int j = 1; j < nn_indices.size (); j++)
          {
            if (!processed.at (nn_indices[j]))
            {
              double norm_b = sqrt (points.channels[nx_idx].values[indices.at (nn_indices[j])] * points.channels[nx_idx].values[indices.at (nn_indices[j])] +
                                    points.channels[ny_idx].values[indices.at (nn_indices[j])] * points.channels[ny_idx].values[indices.at (nn_indices[j])] +
                                    points.channels[nz_idx].values[indices.at (nn_indices[j])] * points.channels[nz_idx].values[indices.at (nn_indices[j])]);
              // [-1;1]
              double dot_p = points.channels[nx_idx].values[indices.at (i)] * points.channels[nx_idx].values[indices.at (nn_indices[j])] +
                             points.channels[ny_idx].values[indices.at (i)] * points.channels[ny_idx].values[indices.at (nn_indices[j])] +
                             points.channels[nz_idx].values[indices.at (i)] * points.channels[nz_idx].values[indices.at (nn_indices[j])];
              if ( acos (dot_p / (norm_a * norm_b)) < region_angle_threshold_)
              {
                processed[nn_indices[j]] = true;
                seed_queue.push_back (nn_indices[j]);
              }
            }
          }

          sq_idx++;
        }

        // If this queue is satisfactory, add to the clusters
        if (seed_queue.size () >= min_pts_per_cluster)
        {
          Region r;
          //r.indices = seed_queue;
          r.indices.resize (seed_queue.size ());
          for (unsigned int j = 0; j < r.indices.size (); j++)
            r.indices[j] = indices.at (seed_queue[j]);
          clusters.push_back (r);
        }
      }

      // Destroy the tree
      delete tree;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
    sortConcaveHull2D (const sensor_msgs::PointCloud &points, const vector<int> &indices, geometry_msgs::Polygon &poly)
    {
      // Create a tree for these points
      cloud_kdtree::KdTree* tree = new cloud_kdtree::KdTreeANN (points, indices);

      vector<int> seed_queue;
      seed_queue.push_back (0);

      // Create a bool vector of processed point indices, and initialize it to false
      vector<bool> processed;
      processed.resize (indices.size (), false);
      vector<int> nn_indices;
      vector<float> nn_distances;

      // Process all points in the indices vector
      int i = 0;
      processed[i] = true;                            // Mark the current point as "processed"
      while (seed_queue.size () < indices.size ())
      {
        tree->nearestKSearch (seed_queue[i], indices.size (), nn_indices, nn_distances);

        for (unsigned int j = 0;  j < nn_indices.size (); j++)
        {
          if (!processed.at (nn_indices[j]))             // Check the closest neighbor
          {
            processed[nn_indices[j]] = true;
            seed_queue.push_back (nn_indices[j]);
            i++;
            break;
          }
        }
      }

      poly.points.resize (seed_queue.size ());
      for (unsigned int i = 0; i < seed_queue.size (); i++)
      {
        poly.points[i].x = points.points.at (indices.at (seed_queue[i])).x;
        poly.points[i].y = points.points.at (indices.at (seed_queue[i])).y;
        poly.points[i].z = points.points.at (indices.at (seed_queue[i])).z;
      }
      // Destroy the tree
      delete tree;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int
      fitSACPlane (sensor_msgs::PointCloud *points, vector<int> *indices, vector<vector<int> > &inliers, vector<vector<double> > &coeff)
    {
      vector<int> empty_inliers;
      vector<double> empty_coeffs;
      if ((int)indices->size () < sac_min_points_per_model_)
      {
        //ROS_ERROR ("fitSACPlane: Indices.size (%d) < sac_min_points_per_model (%d)!", indices->size (), sac_min_points_per_model_);
        inliers.push_back (empty_inliers);
        coeff.push_back (empty_coeffs);
        return (-1);
      }

      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
      sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, sac_distance_threshold_);
      sac->setMaxIterations (120);
      sac->setProbability (0.99);
      model->setDataSet (points, *indices);

      sensor_msgs::PointCloud pts (*points);
      int nr_points_left = indices->size ();
      int nr_models = 0;
      while (nr_points_left > sac_min_points_left_)
      {
        // Search for the best plane
        if (sac->computeModel ())
        {
          // Obtain the inliers and the planar model coefficients
          if ((int)sac->getInliers ().size () < sac_min_points_per_model_)
          {
            //ROS_ERROR ("fitSACPlane: Inliers.size (%d) < sac_min_points_per_model (%d)!", sac->getInliers ().size (), sac_min_points_per_model_);
            inliers.push_back (empty_inliers);
            coeff.push_back (empty_coeffs);
            //return (-1);
            break;
          }
          inliers.push_back (sac->getInliers ());
          vector<double> model_coeff;
          sac->computeCoefficients (model_coeff);
          sac->refineCoefficients  (model_coeff);
          coeff.push_back (model_coeff);

          //fprintf (stderr, "> Found a model supported by %d inliers: [%g, %g, %g, %g]\n", sac->getInliers ().size (),
          //         coeff[coeff.size () - 1][0], coeff[coeff.size () - 1][1], coeff[coeff.size () - 1][2], coeff[coeff.size () - 1][3]);

          // Project the inliers onto the model
          model->projectPointsInPlace (sac->getInliers (), coeff[coeff.size () - 1]);

          // Remove the current inliers in the model
          nr_points_left = sac->removeInliers ();
          nr_models++;
        }
      }
      return (0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      getObjectClassForParallel (sensor_msgs::PointCloud *points, vector<int> *indices, vector<double> *coeff,
                                 geometry_msgs::PointStamped map_origin, double &rgb)
    {
      double r = 0, g = 0, b = 0;
      // Get all planes parallel to the floor (perpendicular to Z)
      geometry_msgs::Point32 robot_origin;
      robot_origin.x = map_origin.point.x;
      robot_origin.y = map_origin.point.y;
      robot_origin.z = map_origin.point.z;

      // Assuming that the robot always has its base at Z = 0 in the base_link frame,
      // compute the X,Y centroid coordinates of the planar patch
      robot_origin.x = robot_origin.y = 0;
      for (unsigned int i = 0; i < indices->size (); i++)
      {
        robot_origin.x += points->points.at (indices->at (i)).x;
        robot_origin.y += points->points.at (indices->at (i)).y;
      }
      robot_origin.x /= indices->size ();
      robot_origin.y /= indices->size ();

      // Compute a distance from 0,0,0 to the plane
      double distance = cloud_geometry::distances::pointToPlaneDistance (robot_origin, *coeff);

      // Test for floor
      if (distance < rule_floor_)
      {
        r = 81; g = 239; b = 168;
      }
      // Test for ceiling
      else if (distance > rule_ceiling_)
      {
        r = 196; g = 197; b = 102;
      }
      // Test for tables
      else if (distance >= rule_table_min_ && distance <= rule_table_max_)
      {
        r = 229; g = 86; b = 35;
      }
      int res = (int(r) << 16) | (int(g) << 8) | int(b);
      rgb = *(float*)(&res);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      getObjectClassForPerpendicular (sensor_msgs::PointCloud *points, vector<int> *indices,
                                      double &rgb)
    {
      double r, g, b;
      // Get the minimum and maximum bounds of the plane
      geometry_msgs::Point32 minP, maxP;
      cloud_geometry::statistics::getMinMax (*points, *indices, minP, maxP);
      // Test for wall
      if (maxP.z > rule_wall_)
      {
        r = rand () / (RAND_MAX + 1.0);
        g = rand () / (RAND_MAX + 1.0);
        b = rand () / (RAND_MAX + 1.0);
        r = r * .3;
        b = b * .3 + .7;
        g = g * .3;

        int res = (int(r * 255) << 16) | (int(g*255) << 8) | int(b*255);
        rgb = *(float*)(&res);
      }
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      computeConcaveHull (const sensor_msgs::PointCloud &points, const vector<int> &indices, const vector<double> &coeff,
                          const vector<vector<int> > &neighbors, geometry_msgs::Polygon &poly)
    {
      Eigen::Vector3d u, v;
      cloud_geometry::getCoordinateSystemOnPlane (coeff, u, v);

      vector<int> inliers (indices.size ());
      int nr_p = 0;
      for (unsigned int i = 0; i < indices.size (); i++)
      {
        if (cloud_geometry::nearest::isBoundaryPoint (points, indices.at (i), neighbors.at (i), u, v, boundary_angle_threshold_))
        {
          inliers[nr_p] = indices.at (i);
          nr_p++;
        }
      }
      inliers.resize (nr_p);

      sortConcaveHull2D (points, inliers, poly);
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb (const sensor_msgs::PointCloudConstPtr &cloud_in )
    {
      cloud_ = *cloud_in;
      geometry_msgs::PointStamped base_link_origin, map_origin;
      base_link_origin.point.x = base_link_origin.point.y = base_link_origin.point.z = 0.0;
      base_link_origin.header.frame_id = "base_link";
      base_link_origin.header.stamp = ros::Time();

      tf_.transformPoint ("base_link", base_link_origin, map_origin);

      ROS_INFO ("Received %d data points. Current robot pose is %g, %g, %g", (int)cloud_.points.size (), map_origin.point.x, map_origin.point.y, map_origin.point.z);

      cloud_annotated_.header = cloud_.header;

      int nx = cloud_geometry::getChannelIndex (cloud_, "nx");
      int ny = cloud_geometry::getChannelIndex (cloud_, "ny");
      int nz = cloud_geometry::getChannelIndex (cloud_, "nz");

      if ( (cloud_.channels.size () < 3) || (nx == -1) || (ny == -1) || (nz == -1) )
      {
        ROS_ERROR ("This PointCloud message does not contain normal information!");
        return;
      }

      timeval t1, t2;
      double time_spent;
      gettimeofday (&t1, NULL);

      // ---[ Select points whose normals are parallel with the Z-axis
      vector<int> indices_z;
      cloud_geometry::getPointIndicesAxisParallelNormals (cloud_, nx, ny, nz, eps_angle_, z_axis_, indices_z);

      // ---[ Select points whose normals are perpendicular to the Z-axis
      vector<int> indices_xy;
      cloud_geometry::getPointIndicesAxisPerpendicularNormals (cloud_, nx, ny, nz, eps_angle_, z_axis_, indices_xy);

      vector<Region> clusters;
      // Split the Z-parallel points into clusters
      findClusters (cloud_, indices_z, region_growing_tolerance_, clusters, nx, ny, nz, min_cluster_pts_);
      int z_c = clusters.size ();
      for (int i = 0; i < z_c; i++)
        clusters[i].region_type = 0;

      // Split the Z-perpendicular points into clusters
      findClusters (cloud_, indices_xy, region_growing_tolerance_, clusters, nx, ny, nz, min_cluster_pts_);
      for (unsigned int i = z_c; i < clusters.size (); i++)
        clusters[i].region_type = 1;

      // Compute the total number of points in all clusters
      int total_p = 0;
      for (int cc = 0; cc < (int)clusters.size (); cc++)
        total_p += clusters[cc].indices.size ();

      gettimeofday (&t2, NULL);
      time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Found %d clusters with %d points in %g seconds.", (int)clusters.size (), total_p, time_spent);
      gettimeofday (&t1, NULL);

      vector<vector<vector<int> > > all_cluster_inliers (clusters.size ());
      vector<vector<vector<double> > > all_cluster_coeff (clusters.size ());

      // Reserve enough space
      cloud_annotated_.points.resize (total_p);
      cloud_annotated_.channels[0].values.resize (total_p);

      int nr_p = 0;

      if (polygonal_map_)
      {
        pmap_.header = cloud_.header;
        pmap_.polygons.resize (clusters.size ());         // Allocate space for the polygonal map
      }

      // Process all clusters in parallel
      #pragma omp parallel for schedule(dynamic)
      for (int cc = 0; cc < (int)clusters.size (); cc++)
      {
        // Find all planes in this cluster
        int ret = fitSACPlane (&cloud_, &clusters[cc].indices, all_cluster_inliers[cc], all_cluster_coeff[cc]);
        if (ret != 0 || all_cluster_inliers[cc].size () == 0)
          continue;
      }

      // Bummer - no omp here
      // cloud_kdtree is not thread safe because we rely on ANN/FL-ANN, so get the neighbors here
      vector<float> nn_distances;
      vector<vector<vector<int> > > neighbors (clusters.size ());
      if (concave_)
      {
        for (int cc = 0; cc < (int)clusters.size (); cc++)
        {
          if (all_cluster_inliers[cc].size () == 0 || all_cluster_coeff[cc].size () == 0)
            continue;

          vector<vector<int> > *cluster_neighbors = &neighbors[cc];

          if (all_cluster_inliers[cc][0].size () == 0)
            continue;
          cluster_neighbors->resize (all_cluster_inliers[cc][0].size ());

          // Create a tree for these points
          cloud_kdtree::KdTree* tree = new cloud_kdtree::KdTreeANN (cloud_, all_cluster_inliers[cc][0]);
          for (unsigned int i = 0; i < all_cluster_inliers[cc][0].size (); i++)
          {
            tree->radiusSearch (i, 0.3, cluster_neighbors->at (i), nn_distances);                      // 30cm radius search
//            tree->nearestKSearch (i, 30);                      // 30cm radius search
            // Note: the neighbors below are in the 0->indices.size () spectrum and need to be
            // transformed into global point indices (!)
            for (unsigned int j = 0; j < cluster_neighbors->at (i).size (); j++)
              cluster_neighbors->at(i).at(j) = all_cluster_inliers[cc][0].at (cluster_neighbors->at(i).at(j));
          }
          // Destroy the tree
          delete tree;
        }
        gettimeofday (&t2, NULL);
        time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
        ROS_INFO ("Nearest neighbors (concave hull) estimated in %g seconds.", time_spent);
        gettimeofday (&t1, NULL);
      }

      if (polygonal_map_)
      {
        // Process all clusters in parallel
        //#pragma omp parallel for schedule(dynamic)
        // Fit convex hulls to the inliers (points should be projected!)
        for (int cc = 0; cc < (int)clusters.size (); cc++)
        {
          if (all_cluster_inliers[cc].size () == 0 || all_cluster_coeff[cc].size () == 0)
            continue;

          if (all_cluster_inliers[cc][0].size () == 0 || all_cluster_coeff[cc][0].size () == 0)
            continue;

          if (concave_)
            computeConcaveHull (cloud_, all_cluster_inliers[cc][0], all_cluster_coeff[cc][0], neighbors[cc], pmap_.polygons[cc]);
          else
            cloud_geometry::areas::convexHull2D (cloud_, all_cluster_inliers[cc][0], all_cluster_coeff[cc][0], pmap_.polygons[cc]);
        }
      }

      // Go over each cluster
      for (int cc = 0; cc < (int)clusters.size (); cc++)
      {
        double r, g, b, rgb;
        r = g = b = 1.0;
        int res = (int(r * 255) << 16) | (int(g*255) << 8) | int(b*255);
        rgb = *(float*)(&res);

        // Get the planes in this cluster
        vector<vector<int> > *planes_inliers   = &all_cluster_inliers[cc];
        vector<vector<double> > *planes_coeffs = &all_cluster_coeff[cc];

        if (planes_inliers->size () == 0 || planes_coeffs->size () == 0 || planes_inliers->size () != planes_coeffs->size ())
          continue;

        // For every plane in this cluster
        for (unsigned int j = 0; j < planes_inliers->size (); j++)
        {
          vector<int> *plane_inliers   = &planes_inliers->at (j);
          vector<double> *plane_coeffs = &planes_coeffs->at (j);

          if (plane_inliers->size () == 0 || plane_coeffs->size () == 0)
            continue;

          // Mark all the points inside
          switch (clusters[cc].region_type)
          {
            case 0:     // Z-parallel
            {
              getObjectClassForParallel (&cloud_, plane_inliers, plane_coeffs, map_origin, rgb);
              break;
            }
            case 1:     // Z-perpendicular
            {
              getObjectClassForPerpendicular (&cloud_, plane_inliers, rgb);
              break;
            }
          }
          for (unsigned int k = 0; k < plane_inliers->size (); k++)
          {
            cloud_annotated_.points[nr_p].x = cloud_.points.at (plane_inliers->at (k)).x;
            cloud_annotated_.points[nr_p].y = cloud_.points.at (plane_inliers->at (k)).y;
            cloud_annotated_.points[nr_p].z = cloud_.points.at (plane_inliers->at (k)).z;
            cloud_annotated_.channels[0].values[nr_p] = rgb;
            nr_p++;
          }
        }
/*        r = rand () / (RAND_MAX + 1.0);
        g = rand () / (RAND_MAX + 1.0);
        b = rand () / (RAND_MAX + 1.0);
        for (unsigned int j = 0; j < clusters[cc].indices.size (); j++)
        {
          cloud_annotated_.points[nr_p].x = cloud_.points.at (clusters[cc].indices.at (j)).x;
          cloud_annotated_.points[nr_p].y = cloud_.points.at (clusters[cc].indices.at (j)).y;
          cloud_annotated_.points[nr_p].z = cloud_.points.at (clusters[cc].indices.at (j)).z;
          cloud_annotated_.channels[0].values[nr_p] = rgb;
          nr_p++;
        }*/
      }

      gettimeofday (&t2, NULL);
      time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Cloud annotated in: %g seconds.", time_spent);
      gettimeofday (&t1, NULL);
      cloud_annotated_.points.resize (nr_p);
      cloud_annotated_.channels[0].values.resize (nr_p);

      cloud_publisher_.publish(cloud_annotated_);

      if (polygonal_map_)
        polygonal_map_publisher_.publish (pmap_);
      return;


/**       ROS_INFO ("Found %d clusters.", clusters.size ());

      // note: the clusters are in the z_axis_indices space so instead of points[clusters[j]] we need to do points[z_axis_indices[clusters[j]]
      nr_p = 0;
      for (unsigned int cc = 0; cc < clusters.size (); cc++)
      {
        double r, g, b;
        r = rand () / (RAND_MAX + 1.0);
        g = rand () / (RAND_MAX + 1.0);
        b = rand () / (RAND_MAX + 1.0);
        for (unsigned int j = 0; j < clusters[cc].size (); j++)
        {
          cloud_annotated_.points[nr_p].x = cloud_.points[xy_axis_indices[clusters[cc].at (j)]].x;
          cloud_annotated_.points[nr_p].y = cloud_.points[xy_axis_indices[clusters[cc].at (j)]].y;
          cloud_annotated_.points[nr_p].z = cloud_.points[xy_axis_indices[clusters[cc].at (j)]].z;
          //cloud_annotated_.channels[0].values[i] = intensity_value;
          cloud_annotated_.channels[0].values[nr_p] = rgb;
          nr_p++;
        }
      }*/

    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv,"semantic_point_annotator");
  ros::NodeHandle ros_node("~");

  SemanticPointAnnotator p (ros_node);
  ros::spin ();

  return (0);
}
/* ]--- */

