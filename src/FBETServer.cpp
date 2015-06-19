/*
author: zcdoyle@hotmail.com,Beihang university
*/
#include <fbet/FBETServer.h>

#define INF 0x7fffffff
using namespace std;
using namespace octomap;
using octomap_msgs::Octomap;

namespace fbet_server
{
    FBETServer::FBETServer(ros::NodeHandle private_nh_)
    : m_nh(),
      m_pointCloudSub(NULL),
      m_tfPointCloudSub(NULL),
      m_octree(NULL),
      m_maxRange(-1.0),
      m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
      m_res(0.05),
      m_treeDepth(0),
      m_maxTreeDepth(0),
      m_probHit(0.7), m_probMiss(0.4),
      m_thresMin(0.12), m_thresMax(0.97),
      m_pointcloudMinZ(-std::numeric_limits<double>::max()),
      m_pointcloudMaxZ(std::numeric_limits<double>::max()),
      m_occupancyMinZ(-std::numeric_limits<double>::max()),
      m_occupancyMaxZ(std::numeric_limits<double>::max()),
      m_filterSpeckles(false), 
      m_compressMap(true),
      lut(16)
      {
          ros::NodeHandle private_nh(private_nh_);
          private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
          private_nh.param("base_frame_id", m_baseFrameId, m_baseFrameId);

          private_nh.param("pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
          private_nh.param("pointcloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);
          private_nh.param("occupancy_min_z", m_occupancyMinZ,m_occupancyMinZ);
          private_nh.param("occupancy_max_z", m_occupancyMaxZ,m_occupancyMaxZ);

          private_nh.param("filter_speckles", m_filterSpeckles, m_filterSpeckles);

          private_nh.param("sensor_model/max_range", m_maxRange, m_maxRange);

          private_nh.param("resolution", m_res, m_res);
          private_nh.param("sensor_model/hit", m_probHit, m_probHit);
          private_nh.param("sensor_model/miss", m_probMiss, m_probMiss);
          private_nh.param("sensor_model/min", m_thresMin, m_thresMin);
          private_nh.param("sensor_model/max", m_thresMax, m_thresMax);
          private_nh.param("compress_map", m_compressMap, m_compressMap);

          logfile.open("log.txt");
          logfile<<"This is a log file for 3D-FBET"<<endl;

          // initialize octomap object & params
          m_octree = new OcTree(m_res);
          m_octree->setProbHit(m_probHit);
          m_octree->setProbMiss(m_probMiss);
          m_octree->setClampingThresMin(m_thresMin);
          m_octree->setClampingThresMax(m_thresMax);
          m_treeDepth = m_octree->getTreeDepth();
          m_maxTreeDepth = m_treeDepth;

          m_color.r = 0.0;
          m_color.g = 0.0;
          m_color.b = 1.0;
          m_color.a = 1.0;

          m_colorFree.r = 0.0;
          m_colorFree.g = 1.0;
          m_colorFree.b = 0.0;
          m_colorFree.a = 1.0;
  
          m_colorFrontier.r = 1.0;
          m_colorFrontier.g = 0.0;
          m_colorFrontier.b = 0.0;
          m_colorFrontier.a = 1.0;

          m_colorgoalFrontier.r = 1.0;
          m_colorgoalFrontier.g = 1.0;
          m_colorgoalFrontier.b = 0.0;
          m_colorgoalFrontier.a = 1.0;

          m_goalposePub = m_nh.advertise<visualization_msgs::Marker>("frontier_goal_marker", 1, false);
          m_markerPub = m_nh.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, false);
          m_fmarkerPub = m_nh.advertise<visualization_msgs::MarkerArray>("free_cells_vis_array", 1, false);
          m_markerPubFro = m_nh.advertise<visualization_msgs::MarkerArray>("frontier_cells_vis_array", 1, false);

          m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "cloud_in", 5);
          m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 5);
          m_tfPointCloudSub->registerCallback(boost::bind(&FBETServer::insertCallback, this, _1));
      }

      FBETServer::~FBETServer()
      {
          if (m_tfPointCloudSub)
          {
              delete m_tfPointCloudSub;
              m_tfPointCloudSub = NULL;
          }

          if (m_pointCloudSub)
          {
              delete m_pointCloudSub;
              m_pointCloudSub = NULL;
          }

         if (m_octree)
         {
             delete m_octree;
             m_octree = NULL;
         }	
      }

      void FBETServer::genNeighborCoord(float x,float y,float z) 
      {
          point3d point(x,y,z);
          OcTreeKey key;
          if (!m_octree->coordToKeyChecked(point, key)) 
          {
	OCTOMAP_ERROR_STR("Error in search: [" << point << "] is out of OcTree bounds!");
	return;
          }
          std::vector<octomap::point3d> neighbor;
          genNeighborCoord(key, neighbor);
      }

      void FBETServer::genNeighborCoord(OcTreeKey start_key, std::vector<octomap::point3d>& occupiedNeighbor) 
      {
          occupiedNeighbor.clear();
          OcTreeKey neighbor_key;
          for (int i = 0; i < 26; i++) 
          {
              lut.genNeighborKey(start_key, i, neighbor_key);
	point3d query = m_octree->keyToCoord(neighbor_key);
	occupiedNeighbor.push_back(query);

          }
      }

      void FBETServer::trackChanges(pcl::PointCloud<pcl::PointXYZI>& changedCells) 
      {
          logfile<<"trackChanges"<<endl;
          KeyBoolMap::const_iterator startPnt = m_octree->changedKeysBegin();
          KeyBoolMap::const_iterator endPnt = m_octree->changedKeysEnd();
  
          int c = 0;
          for (KeyBoolMap::const_iterator iter = startPnt; iter != endPnt; ++iter) 
          {
    	c++;
    	OcTreeNode* node = m_octree->search(iter->first);

    	bool occupied = m_octree->isNodeOccupied(node);

    	pcl::PointXYZI pnt;
   
    	pnt.x = m_octree->keyToCoord(iter->first.k[0]);
    	pnt.y = m_octree->keyToCoord(iter->first.k[1]);
    	pnt.z = m_octree->keyToCoord(iter->first.k[2]);

    	if (occupied) 
    	{
      	    pnt.intensity = 1000;
              }
    	else 
    	{
      	    pnt.intensity = -1000;
    	}
    	changedCells.push_back(pnt);
          }
          m_octree->resetChangeDetection();
      }

      void FBETServer::find_frontier(pcl::PointCloud<pcl::PointXYZI>& changedCells , KeySet& frontierCells)
      {
           ros::WallTime startTime = ros::WallTime::now();
           logfile<<"find_frontier"<<endl;
           frontierCells.clear();
           int flag1,flag2,i;
           std::vector<octomap::point3d> neighbor;
           for(i = 0; i < changedCells.points.size(); i++)
           {
                //get changed point
                float x,y,z;
      	  x = changedCells.points[i].x;
    	  y = changedCells.points[i].y;
    	  z = changedCells.points[i].z;
    	  point3d changed_point(x,y,z);
    	  //transform from point to key
    	  OcTreeKey key;
    	  if (!m_octree->coordToKeyChecked(changed_point, key)) 
    	  {
        	      OCTOMAP_ERROR_STR("Error in search: [" << changed_point << "] is out of OcTree bounds!");
        	      return;
    	  }

    	  //check point state: free/occupied
    	  OcTreeNode* node = m_octree->search(key);
    	  bool occupied = m_octree->isNodeOccupied(node);
    	  if(!occupied)
    	  {
        	      flag1=0;
                    flag2=0;
        	      genNeighborCoord(key, neighbor) ;
        	      for (std::vector<point3d>::iterator iter = neighbor.begin();iter != neighbor.end(); iter++)
        	      {
            	          point3d neipoint=*iter;

            	          //check point state: free/unknown
            	          OcTreeNode* node = m_octree->search(neipoint);
            	          if(node == NULL)
                	flag1=1;
            	          else
            	          {
                	if(!m_octree->isNodeOccupied(node))
            	      	flag2=1;
                        }
        	      }
        	    if(flag1==1 && flag2==1)
        	    {
        	        frontierCells.insert(key);
        	    }
    	  }
          }
          int frontier_size=0;
          for(KeySet::iterator iter = frontierCells.begin(), end=frontierCells.end(); iter!= end; ++iter)
          {
              octomap::point3d fpoint;
              fpoint = m_octree->keyToCoord(*iter);
              logfile<<"fx"<<fpoint.x()<<" "<<"fy"<<fpoint.y()<<" "<<"fz"<<fpoint.z()<<endl;
              frontier_size++;
          }
          logfile<<"frontier:"<<frontier_size<<endl;
          double total_time = (ros::WallTime::now() - startTime).toSec();
          logfile<<"find_frontier used total "<<total_time<<" sec"<<endl;
      }

      void FBETServer::best_frontier(point3d cur_p,KeySet& frontierCells)
      {
    	logfile<<"best_frontier"<<endl;
    	float min_l=INF;
    	float wei_l;
    	int i;
    	point3d best_goal;
    	for(KeySet::iterator iter = frontierCells.begin(), end=frontierCells.end(); iter!= end; ++iter)
    	{
	      point3d fpoint;
	      fpoint = m_octree->keyToCoord(*iter);
	      wei_l=(fpoint.x()-cur_p.x())*(fpoint.x()-cur_p.x())+(fpoint.y()-cur_p.y())*(fpoint.y()-cur_p.y())+(fpoint.z()-cur_p.z())*(fpoint.z()-cur_p.z());
	      if (wei_l<min_l)
	      {
		min_l=wei_l;
		best_goal=fpoint;
	      }
	}
	best_frontiergoal = best_goal;
	logfile<<"bx"<<best_frontiergoal.x()<<" "<<"by"<<best_frontiergoal.y()<<" "<<"bz"<<best_frontiergoal.z()<<endl;
      }

      void FBETServer::insertCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
      {
  	ros::WallTime startTime = ros::WallTime::now();
  	logfile<<"insertCallback"<<endl;

  	PCLPointCloud pc; // input cloud for filtering and ground-detection
  	pcl::fromROSMsg(*cloud, pc);

  	tf::StampedTransform sensorToWorldTf;
  	try 
  	{
   	    m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
  	} catch(tf::TransformException& ex){
    	ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
    	return;
  	}

  	Eigen::Matrix4f sensorToWorld;
  	pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);


  	// set up filter for height range, also removes NANs:
  	pcl::PassThrough<pcl::PointXYZ> pass;
  	pass.setFilterFieldName("z");
  	pass.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

    	// directly transform to map frame:
    	pcl::transformPointCloud(pc, pc, sensorToWorld);

    	// just filter height range:
    	pass.setInputCloud(pc.makeShared());
    	pass.filter(pc);

	insertScan(sensorToWorldTf.getOrigin(), pc);

  	publishAll(cloud->header.stamp);
  	publishfrontier(cloud->header.stamp,frontier_cells);
  	publishfrontiergoal(cloud->header.stamp);
      }

      void FBETServer::insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& cloud)
      {
   	logfile<<"insertScan"<<endl;
  	point3d sensorOrigin = pointTfToOctomap(sensorOriginTf);

  	if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)|| !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
  	{
    		ROS_ERROR_STREAM("Could not generate Key for origin "<<sensorOrigin);
  	}
  	
  	KeySet free_cells, occupied_cells;

  	for (PCLPointCloud::const_iterator it = cloud.begin(); it != cloud.end(); ++it)
  	{
    		point3d point(it->x, it->y, it->z);
    		// maxrange check
    		if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) 
    		{

      			// free cells
      			if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay))
      			{
        				free_cells.insert(m_keyRay.begin(), m_keyRay.end());  
      			}
      			// occupied endpoint
      			OcTreeKey key;
      			if (m_octree->coordToKeyChecked(point, key))
      			{
        				occupied_cells.insert(key);

        				updateMinKey(key, m_updateBBXMin);
        				updateMaxKey(key, m_updateBBXMax);
      			}
    		}
    		else 
    		{// ray longer than maxrange:;
      			point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
      			if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay))
      			{
        				free_cells.insert(m_keyRay.begin(), m_keyRay.end());

        				octomap::OcTreeKey endKey;
        				if (m_octree->coordToKeyChecked(new_end, endKey))
        				{
          					updateMinKey(endKey, m_updateBBXMin);
          					updateMaxKey(endKey, m_updateBBXMax);
        				} 
        				else
        				{
          					ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
        				}


      			}
    		}
  	}

  	// mark free cells only if not seen occupied in this cloud
  	for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it)
  	{
    		if (occupied_cells.find(*it) == occupied_cells.end())
      			m_octree->updateNode(*it, false);
  	}

  	// now mark all occupied cells:
  	for (KeySet::iterator it = occupied_cells.begin(), end=free_cells.end(); it!= end; it++) 
  	{
    		m_octree->updateNode(*it, true);
  	}

  	octomap::point3d minPt, maxPt;
 
  	minPt = m_octree->keyToCoord(m_updateBBXMin);
  	maxPt = m_octree->keyToCoord(m_updateBBXMax);
  
  	if (m_compressMap)
    	m_octree->prune();

  	//extract frontiers
  	//get changed cells changed_cells
  	m_octree->enableChangeDetection(true);
  	pcl::PointCloud<pcl::PointXYZI> changed_cells ;
  	trackChanges(changed_cells);

  	//for every cell in changed_cells check isFrontier
  
  	find_frontier(changed_cells,frontier_cells);
  	best_frontier(sensorOrigin,frontier_cells);
      }

      void FBETServer::publishfrontier(const ros::Time& rostime, KeySet& frontierCells)
      {
  	logfile<<"publishfrontier"<<endl;
  	// init markers for free space:
  	visualization_msgs::MarkerArray frontierNodesVis;
  	// each array stores all cubes of a different size, one for each depth level:
  	frontierNodesVis.markers.resize(m_treeDepth+1);
  
  	for (OcTree::iterator it = m_octree->begin(m_maxTreeDepth),end = m_octree->end(); it != end; ++it)
  	{
        		bool isfron = false;
        		for(KeySet::iterator iter = frontierCells.begin(), end=frontierCells.end(); iter!= end; ++iter)
        		{
            			octomap::point3d fpoint;
            			fpoint = m_octree->keyToCoord(*iter);
            			if(it.getX() == fpoint.x() && it.getY() == fpoint.y() && it.getZ() == fpoint.z() )
                			isfron = true;
        		}
        		if (isfron)
        		{
			double x = it.getX();
             			double y = it.getY();
             			double z = it.getZ();
          
              		unsigned idx = it.getDepth();
              		assert(idx < frontierNodesVis.markers.size());

              		geometry_msgs::Point cubeCenter;
              		cubeCenter.x = x;
              		cubeCenter.y = y;
              		cubeCenter.z = z;

              		frontierNodesVis.markers[idx].points.push_back(cubeCenter);
        		} 
  	}
  	// finish MarkerArray:
    	for (unsigned i= 0; i < frontierNodesVis.markers.size(); ++i)
    	{
      		double size = m_octree->getNodeSize(i);

      		frontierNodesVis.markers[i].header.frame_id = m_worldFrameId;
		frontierNodesVis.markers[i].header.stamp = rostime;
		frontierNodesVis.markers[i].ns = "map";
		frontierNodesVis.markers[i].id = i;
		frontierNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
		frontierNodesVis.markers[i].scale.x = size;
		frontierNodesVis.markers[i].scale.y = size;
		frontierNodesVis.markers[i].scale.z = size;
		frontierNodesVis.markers[i].color = m_colorFrontier;

		if (frontierNodesVis.markers[i].points.size() > 0)
			frontierNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
		else
			frontierNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
	}

    	m_markerPubFro.publish(frontierNodesVis);
      }

      void FBETServer::publishfrontiergoal(const ros::Time& rostime)
      {
    	visualization_msgs::Marker frontier_goal;

	geometry_msgs::Point cubeCenter;
	cubeCenter.x = best_frontiergoal.x();
	cubeCenter.y = best_frontiergoal.y();
	cubeCenter.z = best_frontiergoal.z();

	frontier_goal.points.push_back(cubeCenter);
	double size = m_octree->getNodeSize(m_treeDepth);

	frontier_goal.header.frame_id = m_worldFrameId;
	frontier_goal.header.stamp = rostime;
	frontier_goal.ns = "map";
	frontier_goal.type = visualization_msgs::Marker::CUBE_LIST;
	frontier_goal.scale.x = size;
	frontier_goal.scale.y = size;
	frontier_goal.scale.z = size;
	frontier_goal.color = m_colorgoalFrontier;


	if (frontier_goal.points.size() > 0)
		frontier_goal.action = visualization_msgs::Marker::ADD;
	else
		frontier_goal.action = visualization_msgs::Marker::DELETE;

	m_goalposePub.publish(frontier_goal);
      }

      void FBETServer::publishAll(const ros::Time& rostime)
      {
	ros::WallTime startTime = ros::WallTime::now();
	size_t octomapSize = m_octree->size();
	if (octomapSize <= 1)
	{
	    ROS_WARN("Nothing to publish, octree is empty");
	    return;
	}

	// init markers for free space:
	visualization_msgs::MarkerArray freeNodesVis;
	// each array stores all cubes of a different size, one for each depth level:
	freeNodesVis.markers.resize(m_treeDepth+1);

	// init markers:
	visualization_msgs::MarkerArray occupiedNodesVis;
	// each array stores all cubes of a different size, one for each depth level:
	occupiedNodesVis.markers.resize(m_treeDepth+1);

  	// now, traverse all leafs in the tree:
  	for (OcTree::iterator it = m_octree->begin(m_maxTreeDepth),end = m_octree->end(); it != end; ++it)
  	{
    		if (m_octree->isNodeOccupied(*it))
    		{
      			double z = it.getZ();
      			if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
      			{
        				double size = it.getSize();
        				double x = it.getX();
        				double y = it.getY();

				// Ignore speckles in the map:
				if (m_filterSpeckles && (it.getDepth() == m_treeDepth +1) && isSpeckleNode(it.getKey()))
				{
				  ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
				  continue;
				} // else: current octree node is no speckle, send it out
        				
        				//create marker:
				unsigned idx = it.getDepth();
				assert(idx < occupiedNodesVis.markers.size());

				geometry_msgs::Point cubeCenter;
				cubeCenter.x = x;
				cubeCenter.y = y;
				cubeCenter.z = z;

				occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
      			}
    		} 
    		else
    		{ // node not occupied => mark as free in 2D map if unknown so far
      			double z = it.getZ();
			if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
			{
			
				double x = it.getX();
				double y = it.getY();

				//create marker for free space:
				unsigned idx = it.getDepth();
				assert(idx < freeNodesVis.markers.size());

				geometry_msgs::Point cubeCenter;
				cubeCenter.x = x;
				cubeCenter.y = y;
				cubeCenter.z = z;

				freeNodesVis.markers[idx].points.push_back(cubeCenter);
			}
    		}
  	}

	// finish MarkerArray:
	for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i)
	{
		double size = m_octree->getNodeSize(i);

		occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
		occupiedNodesVis.markers[i].header.stamp = rostime;
		occupiedNodesVis.markers[i].ns = "map";
		occupiedNodesVis.markers[i].id = i;
		occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
		occupiedNodesVis.markers[i].scale.x = size;
		occupiedNodesVis.markers[i].scale.y = size;
		occupiedNodesVis.markers[i].scale.z = size;
		occupiedNodesVis.markers[i].color = m_color;

		if (occupiedNodesVis.markers[i].points.size() > 0)
		occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
		else
		occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
	}
	m_markerPub.publish(occupiedNodesVis);


	// finish FreeMarkerArray:
	for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i)
	{
		double size = m_octree->getNodeSize(i);

		freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
		freeNodesVis.markers[i].header.stamp = rostime;
		freeNodesVis.markers[i].ns = "map";
		freeNodesVis.markers[i].id = i;
		freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
		freeNodesVis.markers[i].scale.x = size;
		freeNodesVis.markers[i].scale.y = size;
		freeNodesVis.markers[i].scale.z = size;
		freeNodesVis.markers[i].color = m_colorFree;

		if (freeNodesVis.markers[i].points.size() > 0)
		freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
		else
		freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
	}
	m_fmarkerPub.publish(freeNodesVis);
      }

      bool FBETServer::isSpeckleNode(const OcTreeKey&nKey) const 
      {
  	OcTreeKey key;
  	bool neighborFound = false;
  	for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]){
    		for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]){
      			for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]){
        				if (key != nKey){
          					OcTreeNode* node = m_octree->search(key);
          					if (node && m_octree->isNodeOccupied(node)){
            						// we have a neighbor => break!
            						neighborFound = true;
          					}
        				}
      			}
    		}
  	}

  	return neighborFound;
      }

}