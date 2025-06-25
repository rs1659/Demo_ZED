#include "CameraZED.hpp"
#include <iostream>

// ZED includes
#include <sl/Camera.hpp>

// Sample includes
#include "TrackingViewer.hpp"

// Using std and sl namespaces
using namespace std;
using namespace sl;
using namespace CameraOperations;

CameraZED::CameraZED(int cameraSerialNumber, bool useSVO)
{   
    this->cameraSerialNumber = cameraSerialNumber;
    this->useSVO = useSVO;

    //Production code would be multithreaded, this was cut in sample for simplicity reasons
    //cameraThread = new std::thread(&CameraZED::cameraTaskFunction, this);
    cameraTaskFunction();
}

CameraZED::~CameraZED()
{
    //cameraThread->join();
}

void CameraZED::cameraTaskFunction()
{
    // Create ZED objects
    Camera zed;
    InitParameters init_parameters;
    init_parameters.camera_resolution = RESOLUTION::HD720;
    init_parameters.camera_fps = 15;
    init_parameters.sdk_verbose = true;

    init_parameters.depth_mode = DEPTH_MODE::PERFORMANCE;
    init_parameters.depth_maximum_distance = 10.0f * 1000.0f;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed
    useSVO ? init_parameters.input.setFromSVOFile("SVO.svo2") : init_parameters.input.setFromSerialNumber(cameraSerialNumber);
    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        sleep(10);
        cameraTaskFunction();
        return;
    }

    // Define the Objects detection module parameters
    ObjectDetectionParameters detection_parameters;
    detection_parameters.enable_tracking = true;
    detection_parameters.enable_segmentation = false; // designed to give person pixel mask
    detection_parameters.detection_model = OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_ACCURATE;

    auto camera_config = zed.getCameraInformation().camera_configuration;

    PositionalTrackingParameters positional_tracking_parameters;
    zed.enablePositionalTracking(positional_tracking_parameters);

    returned_state = zed.enableObjectDetection(detection_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        zed.close();
        sleep(10);
        cameraTaskFunction();
        return;
    }

    // Detection runtime parameters
    int detection_confidence = 60;
    bool showIgnoredObjects = true;
    
    ObjectDetectionRuntimeParameters detection_parameters_rt(60);
    // To select a set of specific object classes:
    detection_parameters_rt.object_class_filter = { OBJECT_CLASS::VEHICLE, OBJECT_CLASS::PERSON };
    // To set a specific threshold
    detection_parameters_rt.object_class_detection_confidence_threshold[OBJECT_CLASS::PERSON] = detection_confidence;
    detection_parameters_rt.object_class_detection_confidence_threshold[OBJECT_CLASS::VEHICLE] = detection_confidence;

    // Detection output
    Objects objects;

    Resolution display_resolution(min((int)camera_config.resolution.width, 1280), min((int)camera_config.resolution.height, 720));
    // create a global image to store both image and tracks view
    cv::Mat global_image(display_resolution.height, display_resolution.width, CV_8UC4);
    // retrieve ref on image part
    auto image_left_ocv = global_image(cv::Rect(0, 0, display_resolution.width, display_resolution.height));

    //The following declarations are required for the compiler to stop complaining
    Resolution tracks_resolution(400, display_resolution.height);
    // init an sl::Mat from the ocv image ref (which is in fact the memory of global_image)
    Mat image_left(display_resolution, MAT_TYPE::U8_C4, image_left_ocv.data, image_left_ocv.step);
    sl::float2 img_scale(display_resolution.width / (float)camera_config.resolution.width, display_resolution.height / (float)camera_config.resolution.height);
    string window_name = "CAM 1";
    char key = ' ';
    Resolution pc_resolution(min((int)camera_config.resolution.width, 720), min((int)camera_config.resolution.height, 404));
    Mat point_cloud(pc_resolution, MAT_TYPE::F32_C4, MEM::GPU);  
   
    cv::namedWindow(window_name, cv::WINDOW_NORMAL); // Create Window
    cv::moveWindow(window_name, 60, 30);
    cv::resizeWindow(window_name, 620, 310);
    auto camera_parameters = zed.getCameraInformation(pc_resolution).camera_configuration.calibration_parameters.left_cam;

    RuntimeParameters runtime_parameters;
    runtime_parameters.confidence_threshold = 20;

    Pose cam_pose;
    cam_pose.pose_data.setIdentity();

    while (true)
    {
        if(zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS)
        {
            // update confidence threshold based on TrackBar
            if (detection_parameters_rt.object_class_filter.empty())
                detection_parameters_rt.detection_confidence_threshold = detection_confidence;
            else // if using class filter, set confidence for each class
                for (auto& it : detection_parameters_rt.object_class_filter)
                    detection_parameters_rt.object_class_detection_confidence_threshold[it] = detection_confidence;

            returned_state = zed.retrieveObjects(objects, detection_parameters_rt);

            if ((returned_state == ERROR_CODE::SUCCESS) && objects.is_new) 
            {
                zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA, MEM::GPU, pc_resolution);
                zed.getPosition(cam_pose, REFERENCE_FRAME::CAMERA);

                zed.retrieveImage(image_left, VIEW::LEFT, MEM::CPU, display_resolution);
                render_2D(image_left_ocv, img_scale, objects.object_list, std::map<int, int>{{0,0}}, showIgnoredObjects, true);

                zed.getPosition(cam_pose, REFERENCE_FRAME::CAMERA);
                        
            }
            cv::imshow(window_name, global_image);
            key = cv::waitKey(10);
        }
        else if(useSVO)
        {
            zed.setSVOPosition(10);         
        }
    }

    //Obviously this part is never reached in this sample code, but production code should also continue running the while indefinitely and never hit any stop conditions
    point_cloud.free();
    image_left.free();

    zed.disableObjectDetection();
 
    zed.close();
}