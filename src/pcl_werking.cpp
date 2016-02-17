    // Create a container for the data.
  sensor_msgs::PointCloud2 output;
  sensor_msgs::PointCloud2 in_points;
  sensor_msgs::PointCloud2 out_points;
  //pcl::PointCloud<pcl::PointXYZ> input_cloud;

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);


  // Do data processing here...
  output = *msg;
  pcl::fromROSMsg (*msg,*input_cloud);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  //seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.05);
    // Create the filtering object
  seg.setInputCloud (input_cloud);
  seg.segment (*inliers, *coefficients); 

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (input_cloud);
  extract.setIndices (inliers);
  
  //extracting Points in
  extract.setNegative (false);
  extract.filter (*cloud_p);
  //extracting Point out
  extract.setNegative (true);
  extract.filter (*cloud_f);
  
  input_cloud.swap (cloud_f);
  pcl::toROSMsg (*cloud_p, in_points);
  pcl::toROSMsg (*cloud_f, out_points);    

  // Publish the data.
 pub_in.publish(in_points);
 pub_out.publish(out_points);