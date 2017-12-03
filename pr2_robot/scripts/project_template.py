#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Here we specify a parameter to let us know which world are we in
# To be honest I am not sure if this could be somehow read in from param server
# I assume yes but I have not figured out the way how to do it yet
# If we figure this out it can be automated even further
world_name = 3

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

def filter_passthrough(pcl, axis, axis_min, axis_max):
    passthrough = pcl.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = axis
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)

    ## Finally use the filter function to obtain the resultant point cloud. 
    return passthrough.filter()

def filter_voxel(poc, size):
    # Voxel Grid Downsampling
    vox = poc.make_voxel_grid_filter()
    # Choose a voxel (also known as leaf) size
    LEAF_SIZE = size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    # Call the filter function to obtain the resultant downsampled point cloud
    return vox.filter()

def filter_ransac_plane(poc, distance):
    # RANSAC Plane Segmentation
    seg = poc.make_segmenter()
    # Set the model you wish to fit 
    # We are fitting a plane -> table
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance 
    # for segmenting the table
    max_distance = distance
    seg.set_distance_threshold(max_distance)

    # Extract inliers and outliers
    # Inliers here means the table which is a little bit unintuitively what we are not interested in
    # Call the segment function to obtain set of inlier indices and model coefficients
    # Extract inliers
    inliers, coefficients = seg.segment()
    extracted_inliers = poc.extract(inliers, negative=False)

    # Extract outliers
    # Save pcd for tabletop objects
    extracted_outliers = poc.extract(inliers, negative=True)
    return (extracted_inliers, extracted_outliers)

def filter_outliers(cloud_filtered, mean, std_dev):
    # Remove noise. It is akin to white noise so it can be effectively removed by means of statistical analysis
    # Much like the previous filters, we start by creating a filter object: 
    outlier_filter = cloud_filtered.make_statistical_outlier_filter()
    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(mean)
    # Set threshold scale factor
    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(std_dev)
    # Finally call the filter function for magic
    return outlier_filter.filter()

# Callback function for your Point Cloud Subscriber
# This function will get called every single time PR2 will capture an image with its camera
# And we will perform all the tricks we learned in Example 2
#  Roughly
#  - removing noise
#  - downsample the voxels so the point cloud takes less memory and is possinble to process faster
#  - perform 3 passes of passthrough carving a box like shape into the space
#  - removing table with RANSAC
#  - clustering with DBCSCAN
# 
#  - performing clssification
#  - constructing messages
#  - either sending them to PR2 or serializing
def pcl_callback(pcl_msg):
    print 'Callback'

# Exercise-2 TODOs:
    # Convert ROS msg to PCL data
    cloud_filtered = ros_to_pcl(pcl_msg)

    # Implement pipeline
    # I moved it to functions to clean it up a bit but
    # here is so much it could be done. This almost calls for a framework like pytoolz
    # where we could build chained pipelines much more naturally without the clutter of
    # intermediary variables

    cloud_filtered = filter_outliers(cloud_filtered, 10, 0.1)
    # Remove noise. It is akin to white noise so it can be effectively removed by means of statistical analysis
    # Much like the previous filters, we start by creating a filter object: 
    outlier_filter = cloud_filtered.make_statistical_outlier_filter()
    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(10)
    # Set threshold scale factor
    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(0.1)
    # Finally call the filter function for magic
    cloud_filtered = outlier_filter.filter()

    # Voxel Grid Downsampling
    cloud_filtered = filter_voxel(cloud_filtered, 0,005)

    # PassThrough Filtering
    # We make three filters along each axis
    cloud_filtered = filter_passthrough(cloud_filtered, 'z', 0.6, 1.1)
    cloud_filtered = filter_passthrough(cloud_filtered, 'y', -0.45, 0.45)
    cloud_filtered = filter_passthrough(cloud_filtered, 'x', 0.35, 0.85)

    extracted_inliers, extracted_outliers =  filter_ransac_plane(cloud_filtered, 0.01)


    # Now we have point cloud representing objects, lets cluster them into separate clumps
    # And apply some colors so we can color them in RViz
    
    # Euclidean Clustering (DBSCAN)
    white_cloud = XYZRGB_to_XYZ(extracted_outliers)
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(2500)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    # Here we create list of lists that we will transform to point cloud shortly, adding a color
    # iterate over cluster
    for j, indices in enumerate(cluster_indices):
        # iterate over indices
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    # pc after pipeline without RANSAC
    ros_cloud_filtered = pcl_to_ros(cloud_filtered)
    # objects
    ros_cloud_objects = pcl_to_ros(extracted_outliers)
    # table
    ros_cloud_table = pcl_to_ros(extracted_inliers)
    # pc of the clouds. Take note this is one big PC, just colored
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # publish the clouds
    my_cloud_pub.publish(ros_cloud_filtered)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_cluster_pub.publish(ros_cluster_cloud)
    
    # Exercise-3
    # Now we are goint to classify
    # Bascially we walk through clusters
    # compute the histograms which is what we trained on
    # and feed this into the classifier
    detected_objects_labels = []
    detected_objects = []
    # Classify the clusters! (loop through each detected cluster one at a time)
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the one object from the extracted outliers (cloud_objects)
        pcl_cluster = extracted_outliers.extract(pts_list)
        # convert the cluster from pcl to ROS using helper function
    	sample_cloud = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        chists = compute_color_histograms(sample_cloud, using_hsv=True)
        normals = get_normals(sample_cloud)
	    nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        # Seems like the label is just made as a shifted position of the first pixes
        label_pos = list(white_cloud[pts_list[0]])
        # shifted in z axis
        label_pos[2] += .4
        # and published to ros
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        # create a datastructure to hold the details of detected object and push to an array
        do = DetectedObject()
        do.label = label
        do.cloud = sample_cloud
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):
    # here we bascially just do a bit data munging to make sure that we
    # generate the message for the robot to pick up
    # the gist is to read in the file where is specified the order how the things 
    # should be picked up
    # and then lookup some other stuff from couple other places

    # Initialize variables
    labels = []
    groups = []
    centroids = []
    # Get/Read parameters from param server
    # did not figure out how to read the world we are in
    object_list_param = rospy.get_param('/object_list')
    dropbox = rospy.get_param('/dropbox')

    # Parse parameters into individual variables
    dropbox_lookup = dict(map(lambda i: [i['group'], i['position']], dropbox))
    arm_lookup = {'red': 'left', 'green': 'right'}

    # Loop through the pick list
    # and build the lists of things. Lables, centroid and groups
    for item in object_list_param:
        # Get the PointCloud for a given object and obtain it's centroid
        name = item['name']
	    object = filter(lambda o: o.label == name, object_list)[0]
        labels.append(name)
        groups.append(item['group'])
	    points_arr = ros_to_pcl(object.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])
 
    # we need to retype the centroids to a different datatype that ROS can understand
    centroids = map(lambda centroid: map(np.asscalar, centroid), centroids)

    dict_list = []    
    for (centroid, label, group) in zip(centroids, labels, groups):

    	test_scene_num = Int32()
    	test_scene_num.data = world_name

    	object_name = String()
    	object_name.data = label

    	arm_name = String()
        arm_name.data = arm_lookup[group]

        pick_pose = Pose()
        pick_pose.position.x = centroid[0]
        pick_pose.position.y = centroid[1]
        pick_pose.position.z = centroid[2]

        place_pos_data = dropbox_lookup[group]
        place_pose = Pose()
        place_pose.position.x = place_pos_data[0]
        place_pose.position.y = place_pos_data[1]
        place_pose.position.z = place_pos_data[2]

	    yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)
    

# Uncomment this if you want to see the robot doing work
# Unfortunately it does not work very often
# he just does not pick up things most of the time

	# TODO: Create 'place_pose' for the object
        
        # TODO: Assign the arm to be used for pick_place

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format

        # Wait for 'pick_place_routine' service to come up


#        rospy.wait_for_service('pick_place_routine')
#        try:
#            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

#            # TODO: Insert your message variables to be sent as a service request
#            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

#            print ("Response: ",resp.success)

#        except rospy.ServiceException, e:
#            print "Service call failed: %s"%e

    # Output your request parameters into output yaml file
    # once we collect all messages we need
    send_to_yaml('yaml_filename_' + str(world_name) + '.yaml', dict_list)


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    
    # Load Model From disk
    model = pickle.load(open('model_' + str(world_name) + '.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    my_cloud_pub = rospy.Publisher("/my_cloud", PointCloud2, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # Initialize color_list
    get_color_list.color_list = []
    print 'spinning'
    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
