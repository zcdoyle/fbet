#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeLUT.h>

#include <vector>
#include <stdio.h>
#include <fstream>

using namespace std;
using namespace octomap;
using octomap_msgs::Octomap;

namespace fbet_server
{
    class FBETServer
    {
        public:
        	typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;

        	FBETServer(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
        	virtual ~FBETServer();

        	virtual void insertCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);

        protected:
        	inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min)
        	{
        	    for (unsigned i=0; i<3; ++i)
        	    min[i] = std::min(in[i], min[i]);
              };
  
  	inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max)
  	{
    	    for (unsigned i=0; i<3; ++i)
      	    max[i] = std::max(in[i], max[i]);
  	};

  	void publishAll(const ros::Time& rostime = ros::Time::now());

  	virtual void insertScan(const tf::Point& sensorOrigin,  const PCLPointCloud& cloud);

  	bool isSpeckleNode(const octomap::OcTreeKey& key) const;

  	void trackChanges(pcl::PointCloud<pcl::PointXYZI>& changedCells);
  	void genNeighborCoord(float x,float y,float z);
  	void genNeighborCoord(octomap::OcTreeKey start_key, std::vector<octomap::point3d>& occupiedNeighbor);
  	void find_frontier(pcl::PointCloud<pcl::PointXYZI>& changedCells , KeySet& frontierCells);
  	void publishfrontier(const ros::Time& rostime, KeySet& frontierCells);
  	void best_frontier(point3d sensorOrigin,KeySet& Cells);
  	void publishfrontiergoal(const ros::Time& rostime);
      void cluster_frontier(KeySet& frontierCells,KeySet& candidateCells);
      void find_center(std::vector<OcTreeKey>& cluster, OcTreeKey& centerCell);
  	ros::NodeHandle m_nh;
  	ros::Publisher  m_markerPub, m_fmarkerPub,m_markerPubFro, m_goalposePub;
  	message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointCloudSub;
  	tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPointCloudSub;
  	tf::TransformListener m_tfListener;

  	octomap::OcTree* m_octree;
  	octomap::KeyRay m_keyRay;  // temp storage for ray casting
      octomap::KeyRay m_keyRaysphere;
  	octomap::OcTreeKey m_updateBBXMin;
  	octomap::OcTreeKey m_updateBBXMax;

  	double m_maxRange;
  	std::string m_worldFrameId; // the map frame
  	std::string m_baseFrameId; // base of the robot for ground plane filtering
  	std_msgs::ColorRGBA m_color;
  	std_msgs::ColorRGBA m_colorFree;
  	std_msgs::ColorRGBA m_colorFrontier;
  	std_msgs::ColorRGBA m_colorgoalFrontier;

  	double m_res;
	unsigned m_treeDepth;
	unsigned m_maxTreeDepth;
	double m_probHit;
	double m_probMiss;
	double m_thresMin;
	double m_thresMax;

	double m_pointcloudMinZ;
	double m_pointcloudMaxZ;
	double m_occupancyMinZ;
	double m_occupancyMaxZ;
	double m_minSizeX;
	double m_minSizeY;
	bool m_filterSpeckles;

	bool m_compressMap;

	ros::Publisher pubFreeChangeSet;
	ros::Publisher pubChangeSet;
	ros::Subscriber subChangeSet;
	ros::Subscriber subFreeChanges;

	KeySet frontier_cells;
       KeySet candidate_cells;
	octomap::OcTreeLUT lut;
	ofstream logfile;
	point3d best_frontiergoal;
    };
}