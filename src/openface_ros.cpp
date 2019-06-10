#include "openface_ros/openface_ros.h"
#include "std_msgs/Int64.h"
#include "openface_ros/face_info.h"
#include "openface_ros/landmark.h"
#include <string>

// #include <boost/circular_buffer.hpp>

using namespace std;
using namespace cv;

OpenFaceRos::OpenFaceRos(string name, double _fx, double _fy, double _cx, double _cy, double _threshold, bool _enable_AU) : 
                         args(1, "openface_ros"), n(name), it(n), det_parameters(args), face_model(det_parameters.model_location),
                         face_analysis_params(args), face_analyser(face_analysis_params), visualizer(true, false, false, false), 
                         gazeDirection0(0, 0, -1), gazeDirection1(0, 0, -1), gazeAngle(0, 0), pupil_left(0, 0, 0), pupil_right(0, 0, 0),
                         pose_estimate(0, 0, 0, 0, 0, -1)
{
    head_status_pub = n.advertise<std_msgs::Int64>("/" + name + "/hiro/lookat_screen", 10);
    gripper_status_pub = n.advertise<std_msgs::Int64>("/" + name + "/hiro/lookat_gripper", 10);
    color_image_sub = it.subscribe("/" + name + "/color/image_raw", 1, &OpenFaceRos::colorCb, this);
    depth_image_sub = it.subscribe("/" + name + "/aligned_depth_to_color/image_raw", 1, &OpenFaceRos::depthCb, this);
    fx = _fx;
    fy = _fy;
    cx = _cx;
    cy = _cy;
    enable_AU = _enable_AU;
    threshold = _threshold;
    cv_depth_valid = 0;
    detection_success = false;
    distance_head = -1;
    distance_gripper = -1;

    //TODO: Delete this
   // boost::circular_buffer<float> buffer(150);
    test = n.advertise<openface_ros::face_info>("/" + name + "/face_info", 10);
}

OpenFaceRos::~OpenFaceRos()
{

}

void OpenFaceRos::calculatePupil(Point3f& pupil_left, Point3f& pupil_right, const vector<Point3f>& eye_landmarks3d)
{
    for (size_t i = 0; i < 8; ++i)
    {
        pupil_left = pupil_left + eye_landmarks3d[i];
        pupil_right = pupil_right + eye_landmarks3d[i + eye_landmarks3d.size()/2];
    }
    pupil_left = pupil_left / 8;
    pupil_right = pupil_right / 8;
}

vector<float> OpenFaceRos::realDistanceTransform(float distance_x, float distance_y, float depth)
{   
    vector<float> real;
    real.push_back( (distance_x)*depth/fx );
    real.push_back( (distance_y)*depth/fy );
    return real;
}

double OpenFaceRos::distanceOfPointToLine(vector<float> a, vector<float> b, vector<float> s) 
{ 
	double ab = sqrt(pow((a[0] - b[0]), 2.0) + pow((a[1] - b[1]), 2.0) + pow((a[2] - b[2]), 2.0));
	double as = sqrt(pow((a[0] - s[0]), 2.0) + pow((a[1] - s[1]), 2.0) + pow((a[2] - s[2]), 2.0));
	double bs = sqrt(pow((s[0] - b[0]), 2.0) + pow((s[1] - b[1]), 2.0) + pow((s[2] - b[2]), 2.0));
	double cos_A = (pow(as, 2.0) + pow(ab, 2.0) - pow(bs, 2.0)) / (2 * ab*as);
	double sin_A = sqrt(1 - pow(cos_A, 2.0));
	return as*sin_A; 
}

Matx33f OpenFaceRos::Euler2RotationMatrix(const Vec3f& eulerAngles)
{
    Matx33f rotation_matrix;

    float s1 = sin(eulerAngles[0]);
    float s2 = sin(eulerAngles[1]);
    float s3 = sin(eulerAngles[2]);

    float c1 = cos(eulerAngles[0]);
    float c2 = cos(eulerAngles[1]);
    float c3 = cos(eulerAngles[2]);

    rotation_matrix(0, 0) = c2 * c3;
    rotation_matrix(0, 1) = -c2 *s3;
    rotation_matrix(0, 2) = s2;
    rotation_matrix(1, 0) = c1 * s3 + c3 * s1 * s2;
    rotation_matrix(1, 1) = c1 * c3 - s1 * s2 * s3;
    rotation_matrix(1, 2) = -c2 * s1;
    rotation_matrix(2, 0) = s1 * s3 - c1 * c3 * s2;
    rotation_matrix(2, 1) = c3 * s1 + c1 * s2 * s3;
    rotation_matrix(2, 2) = c1 * c2;

    return rotation_matrix;
}

void OpenFaceRos::Project(Mat_<float>& dest, const Mat_<float>& mesh, float _fx, float _fy, float _cx, float _cy)
{
    dest = Mat_<float>(mesh.rows, 2, 0.0);

    int num_points = mesh.rows;

    float X, Y, Z;

    Mat_<float>::const_iterator mData = mesh.begin();
    Mat_<float>::iterator projected = dest.begin();

    for (int i = 0; i < num_points; i++)
    {
        // Get the points
        X = *(mData++);
        Y = *(mData++);
        Z = *(mData++);

        float x;
        float y;

        // if depth is 0 the projection is different
        if (Z != 0)
        {
            x = ((X * _fx / Z) + _cx);
            y = ((Y * _fy / Z) + _cy);
        }
        else
        {
            x = X;
            y = Y;
        }

        // Project and store in dest matrix
        (*projected++) = x;
        (*projected++) = y;
    }
}

unsigned short OpenFaceRos::getMedianDepth(int row, int col)
{
    int window_size = 5;
    int half_window_size = window_size/2;
    if ((row - half_window_size < 0 || row + half_window_size >= 480) || (col - half_window_size < 0 || col + half_window_size >= 640))
    {
        return cv_depth_ptr->image.at<unsigned short>(cv::Point(row, col));
    }
    vector<unsigned short> _depth;
    for (int i = row - half_window_size; i <= row + half_window_size; i++)
    {
        for (int j = col - half_window_size; j <= col + half_window_size; j++)
        {
            _depth.push_back(cv_depth_ptr->image.at<unsigned short>(cv::Point(i, j)));
        }
    }
    nth_element(_depth.begin(), _depth.begin() + window_size*window_size/2, _depth.end());
    return _depth[window_size*window_size/2];
}

void OpenFaceRos::colorCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_color_ptr;
    try
    {
        cv_color_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    faceDetection(cv_color_ptr);
}

void OpenFaceRos::faceDetection(cv_bridge::CvImagePtr cv_color_ptr)
{
    Mat grayscale_image;
    Mat rgb_image = cv_color_ptr->image;
    cv::cvtColor(rgb_image, grayscale_image, cv::COLOR_BGR2GRAY);
    detection_success = LandmarkDetector::DetectLandmarksInVideo(rgb_image, face_model, det_parameters, grayscale_image);
    
    // If tracking succeeded and we have an eye model, estimate gaze
    if (detection_success && face_model.eye_model)
    {
        GazeAnalysis::EstimateGaze(face_model, gazeDirection0, fx, fy, cx, cy, true);
        GazeAnalysis::EstimateGaze(face_model, gazeDirection1, fx, fy, cx, cy, false);
        // NEW
        gazeAngle = GazeAnalysis::GetGazeAngle(gazeDirection0, gazeDirection1);
        calculatePupil(pupil_left, pupil_right, LandmarkDetector::Calculate3DEyeLandmarks(face_model, fx, fy, cx, cy));
    }

    //cv::Vec2d gazeAngle(0, 0);
    // gazeAngle = GazeAnalysis::GetGazeAngle(gazeDirection0, gazeDirection1);

    // Work out the pose of the head from the tracked model
    pose_estimate = LandmarkDetector::GetPose(face_model, fx, fy, cx, cy);
    vector<Point> nose = getNose();
    if (detection_success && face_model.eye_model)
    {
        line(rgb_image, nose[0], nose[1], Scalar(110, 220, 0), 2);
    }
    
    // Displaying the tracking visualizations
    visualizer.SetImage(rgb_image, fx, fy, cx, cy);
    visualizer.SetObservationLandmarks(face_model.detected_landmarks, face_model.detection_certainty, face_model.GetVisibilities());
    visualizer.SetObservationPose(pose_estimate, face_model.detection_certainty);
    visualizer.SetObservationGaze(gazeDirection0, gazeDirection1, LandmarkDetector::CalculateAllEyeLandmarks(face_model), LandmarkDetector::Calculate3DEyeLandmarks(face_model, fx, fy, cx, cy), face_model.detection_certainty);
    
    if (enable_AU) 
    {
        Mat sim_warped_img;
        Mat_<double> hog_descriptor; 
        int num_hog_rows = 0, num_hog_cols = 0;    
        face_analyser.AddNextFrame(rgb_image, face_model.detected_landmarks, face_model.detection_success, ros::Time::now().toSec(), true);
        face_analyser.GetLatestAlignedFace(sim_warped_img);
        face_analyser.GetLatestHOG(hog_descriptor, num_hog_rows, num_hog_cols);
        vector<pair<string, double>> face_actions_class = face_analyser.GetCurrentAUsClass();
        visualizer.SetObservationActionUnits(face_analyser.GetCurrentAUsReg(), face_actions_class);
        checkAU(face_actions_class);
    }

    char character_press = visualizer.ShowObservation();
    // restart the tracker
    if (character_press == 'r')
    {
        face_model.Reset();
    }

    if (cv_depth_valid == 1)
    {
        unsigned short depth_left_tmp = cv_depth_ptr->image.at<unsigned short>(cv::Point(pupil_left.x + cx, pupil_left.y + cy));
        depth_left = (float)depth_left_tmp * 0.001;
        unsigned short depth_right_tmp = cv_depth_ptr->image.at<unsigned short>(cv::Point(pupil_right.x + cx, pupil_right.y + cy));
        depth_right = (float)depth_right_tmp * 0.001;
    }
    vector<float> real_pupil_left = realDistanceTransform(pupil_left.x, pupil_left.y, depth_left);
    vector<float> real_pupil_left_tmp{real_pupil_left[0]-gazeDirection0.x, real_pupil_left[1]-gazeDirection0.y, real_pupil_left[2]-gazeDirection0.z};
    vector<float> real_pupil_right = realDistanceTransform(pupil_right.x, pupil_right.y, depth_right);

    if (cv_depth_valid == 1 && detection_success)
    {
        unsigned short depth_nose_tmp = getMedianDepth(nose[0].x, nose[0].y);
        float depth_nose = (float)depth_nose_tmp * 0.001;
        vector<float> real_nose = realDistanceTransform(nose[0].x - cx, nose[0].y - cy, depth_nose);
        tf::Transform head_transform;
        head_transform.setOrigin(tf::Vector3(depth_nose, -real_nose[0], -real_nose[1]));
        tf::Quaternion q;
        q.setRPY(-pose_estimate[4], -pose_estimate[3] - M_PI_2, pose_estimate[5]);
        head_transform.setRotation(q);
        broadcaster.sendTransform(tf::StampedTransform(head_transform, ros::Time::now(), "camera_link", "detected_head"));
    }
    tf::StampedTransform transform;

    distance_head = -1;
    distance_gripper = -1;
    std_msgs::Int64 msgs;
    try
    {
        if (detection_success)
        {
            listener.lookupTransform("/detected_head", "/screen", ros::Time(0), transform);
            distance_head = sqrt(pow(transform.getOrigin().y(), 2) + pow(transform.getOrigin().x(),2));
            listener.lookupTransform("/detected_head", "/stp_021808TP00080_tip", ros::Time(0), transform);
            distance_gripper = sqrt(pow(transform.getOrigin().y(), 2) + pow(transform.getOrigin().x(),2));
        }
    }
    catch (tf::TransformException ex)
    {

    }

    // if (detection_success) {
    //     vector<Point> nose = getNose()
    //     buffer.push_back(nose.y);

    //     if (buffer.full()) {
    //         if (checkHeadNod() == 1) {
    //             std::cout << "Head Nod" << std::endl;
    //         }
    //     }
    // }

    recordFaceInfo();
    
    checkGaze();
}

// int OpenFaceRos::checkHeadNod() 
// {
//     float mean = (std::accumulate(buffer.begin(), buffer.end(), 0)) / buffer.size();

//     float dev_sum = 0.0;
//     float max = -1000.0;
//     float min = 1000.0;

//     for (boost::circular_buffer<float>::iterator it = buffer.begin(); it != buffer.end(); it++) {
//         dev_sum = dev_sum + pow((*it - mean), 2.0);
//         if (it < min)
//             min = it;
//         if (it > max)
//             max = it;
//     }

//     float sd = sqrt(dev_sum / buffer.size(), 2.0);

// } 

void OpenFaceRos::recordFaceInfo() 
{
    openface_ros::face_info face_info_msg;
    face_info_msg.header.stamp = ros::Time::now();
    face_info_msg.num = 0;

    if(detection_success) {
        face_info_msg.num = 1;

        face_info_msg.landmarks.clear();

        for (int i = 0; i < face_model.GetShape(fx, fy, cx, cy).cols; i++) {
            openface_ros::landmark landmark_point;
            landmark_point.ID = i;
            landmark_point.x = face_model.GetShape(fx, fy, cx, cy).at<float>(i, 0);
            landmark_point.y = face_model.GetShape(fx, fy, cx, cy).at<float>(i, 1);
            landmark_point.z = face_model.GetShape(fx, fy, cx, cy).at<float>(i, 2);
            face_info_msg.landmarks.push_back(landmark_point);
        }

        face_info_msg.head_pose.x = pose_estimate[0];
        face_info_msg.head_pose.y = pose_estimate[1];
        face_info_msg.head_pose.z = pose_estimate[2];
        face_info_msg.head_pose.rot_x = pose_estimate[3];
        face_info_msg.head_pose.rot_y = pose_estimate[4];
        face_info_msg.head_pose.rot_z = pose_estimate[5];

        face_info_msg.gazeDirection0.x = gazeDirection0.x;
        face_info_msg.gazeDirection0.y = gazeDirection0.y;
        face_info_msg.gazeDirection0.z = gazeDirection0.z;

        face_info_msg.gazeDirection1.x = gazeDirection1.x;
        face_info_msg.gazeDirection1.y = gazeDirection1.y;
        face_info_msg.gazeDirection1.z = gazeDirection1.z;

        face_info_msg.gazeAngle.x = (float) gazeAngle[0];
        face_info_msg.gazeAngle.y = (float) gazeAngle[1];
        face_info_msg.gazeAngle.z = 0;
    }

    test.publish(face_info_msg);
}

vector<Point> OpenFaceRos::getNose()
{
    Vec3f nose = cv::Vec3f(pose_estimate[0], pose_estimate[1], pose_estimate[2]);
    Matx33f rot = Euler2RotationMatrix(cv::Vec3f(pose_estimate[3], pose_estimate[4], pose_estimate[5]));
    float boxVerts[] = { 0, 0, -1};
    Mat_<float> box = cv::Mat(1, 3, CV_32F, boxVerts).clone();
    Mat_<float> rotBox;
    rotBox = cv::Mat(rot) * box.t();
    rotBox = rotBox.t();
    cv::Mat_<float> nose_direction = rotBox.rowRange(0, 1);
    Mat_<float> proj_points;
    Vec3f nose_direction_new = cv::Vec3f(nose_direction.at<float>(0), nose_direction.at<float>(1), nose_direction.at<float>(2));
    Mat_<float> mesh_0 = (cv::Mat_<float>(2, 3) << nose[0], nose[1], nose[2], nose[0]+nose_direction_new[0]*70.0, nose[1]+nose_direction_new[1]*70.0, nose[2]+nose_direction_new[2]*70.0);
    Project(proj_points, mesh_0, fx, fy, cx, cy);
    vector<Point> nose_2d{ Point( cvRound(proj_points.at<float>(0, 0)), cvRound(proj_points.at<float>(0, 1)) ), Point( cvRound(proj_points.at<float>(1, 0)), cvRound(proj_points.at<float>(1, 1)) )};
    return nose_2d;
}

vector<Point> OpenFaceRos::getLeftPupil()
{
    Vec3f leftPupil = cv::Vec3f(pupil_left.x, pupil_left.y, pupil_left.z);
    Matx33f rot = Euler2RotationMatrix(cv::Vec3f(gazeDirection0.x, gazeDirection0.y, gazeDirection0.z));
    float boxVerts[] = { 0, 0, -1};
    Mat_<float> box = cv::Mat(1, 3, CV_32F, boxVerts).clone();
    Mat_<float> rotBox;
    rotBox = cv::Mat(rot) * box.t();
    rotBox = rotBox.t();
    cv::Mat_<float> gaze_direction = rotBox.rowRange(0, 1);
    Mat_<float> proj_points;
    Vec3f gaze_direction_new = cv::Vec3f(gaze_direction.at<float>(0), gaze_direction.at<float>(1), gaze_direction.at<float>(2));
    Mat_<float> mesh_0 = (cv::Mat_<float>(2, 3) << leftPupil[0], leftPupil[1], leftPupil[2], leftPupil[0]+gaze_direction_new[0]*70.0, leftPupil[1]+gaze_direction_new[1]*70.0, leftPupil[2]+gaze_direction_new[2]*70.0);
    Project(proj_points, mesh_0, fx, fy, cx, cy);
    vector<Point> left_pupil_2d{ Point( cvRound(proj_points.at<float>(0, 0)), cvRound(proj_points.at<float>(0, 1)) ), Point( cvRound(proj_points.at<float>(1, 0)), cvRound(proj_points.at<float>(1, 1)) )};
    return left_pupil_2d;
}

vector<Point> OpenFaceRos::getRightPupil()
{
    Vec3f rightPupil = cv::Vec3f(pupil_right.x, pupil_right.y, pupil_right.z);
    Matx33f rot = Euler2RotationMatrix(cv::Vec3f(gazeDirection1.x, gazeDirection1.y, gazeDirection1.z));
    float boxVerts[] = {0, 0, -1};
    Mat_<float> box = cv::Mat(1, 3, CV_32F, boxVerts).clone();
    Mat_<float> rotBox;
    rotBox = cv::Mat(rot) * box.t();
    rotBox = rotBox.t();
    cv::Mat_<float> gaze_direction = rotBox.rowRange(0, 1);
    Mat_<float> proj_points;
    Vec3f gaze_direction_new = cv::Vec3f(gaze_direction.at<float>(0), gaze_direction.at<float>(1), gaze_direction.at<float>(2));
    Mat_<float> mesh_0 = (cv::Mat_<float>(2, 3) << rightPupil[0], rightPupil[1], rightPupil[2], rightPupil[0]+gaze_direction_new[0]*70.0, rightPupil[1]+gaze_direction_new[1]*70.0, rightPupil[2]+gaze_direction_new[2]*70.0);
    Project(proj_points, mesh_0, fx, fy, cx, cy);
    vector<Point> right_pupil_2d{ Point( cvRound(proj_points.at<float>(0, 0)), cvRound(proj_points.at<float>(0, 1)) ), Point( cvRound(proj_points.at<float>(1, 0)), cvRound(proj_points.at<float>(1, 1)) )};
    return right_pupil_2d;
}

void OpenFaceRos::checkAU(vector<pair<string, double>> face_actions_class)
{
    if (enable_AU)
    {   
        // // print out AUs
        // for (auto au : face_actions_class)
        // {
        //     std::cout << au.first << ": " << au.second << std::endl;
        // }

        if (face_actions_class[0].second == 1)
        {
            std::cout << face_actions_class[0].first << std::endl;
            std::cout << "Angry!!!" << std::endl;
        }
        else if (face_actions_class[2].second == 1 && face_actions_class[5].second == 1)
        {
            std::cout << face_actions_class[2].first << ", " << face_actions_class[5].first << std::endl;
            std::cout << "Smile!!!" << std::endl;
        }
    }
}

void OpenFaceRos::checkGaze()
{   
    std_msgs::Int64 msgs;
    if (distance_head != -1)
    {
        if(distance_head < threshold)
        {
            ROS_INFO("Looking at head!!!");
            msgs.data = 1;
            head_status_pub.publish(msgs);
        }
        else 
        {
            msgs.data = -1;
            head_status_pub.publish(msgs);
        }
    }
    else
    {
        msgs.data = -1;
        head_status_pub.publish(msgs);
    }
    
    if (distance_gripper != -1)
    {
        if(distance_gripper < threshold)
        {
            ROS_INFO("Looking at gripper!!!");
            msgs.data = 1;
            gripper_status_pub.publish(msgs);
        }
        else 
        {
            msgs.data = 0;
            gripper_status_pub.publish(msgs);
        }
    }
    else
    {
        msgs.data = 0;
        gripper_status_pub.publish(msgs);
    }
}

void OpenFaceRos::depthCb(const sensor_msgs::ImageConstPtr& msg)
{
    // cv::namedWindow(DEPTH_OPENCV_WINDOW);
    try
    {
        cv_depth_valid = 1;
        cv_depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        cv_depth_valid = 0;
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // cv::imshow(DEPTH_OPENCV_WINDOW, cv_depth_ptr->image);arguments
    // cv::waitKey(1);
}

vector<string> OpenFaceRos::get_arguments(int argc, char **argv)
{
    vector<std::string> arguments;
    for (int i = 0; i < argc; ++i)
    {
        arguments.push_back(std::string(argv[i]));
    }
    return arguments;
}
