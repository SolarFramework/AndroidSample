#include "CV_Main.h"

#include <iostream>

#include <boost/log/core.hpp>

using namespace std;
#include "SolARModuleOpencv_traits.h"

#include "SolARModuleTools_traits.h"

#include "xpcf/xpcf.h"

#include "api/display/IImageViewer.h"
#include "api/input/devices/ICamera.h"
#include "api/input/files/IMarker2DNaturalImage.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/pose/I2DTransformFinder.h"
#include "api/solver/pose/IHomographyValidation.h"
#include "api/features/IKeypointsReIndexer.h"
#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"
#include "api/display/I2DOverlay.h"
#include "api/display/I3DOverlay.h"
#include "api/geom/IImage2WorldMapper.h"
#include "api/geom/I2DTransform.h"

#include "api/input/files/IMarker2DSquaredBinary.h"
#include "api/image/IImageFilter.h"
#include "api/image/IImageConvertor.h"
#include "api/features/IContoursExtractor.h"
#include "api/features/IContoursFilter.h"
#include "api/image/IPerspectiveController.h"
#include "api/features/IDescriptorsExtractorSBPattern.h"
#include "api/features/ISBPatternReIndexer.h"

#include "SolAROpenCVHelper.h"

#include "SharedBuffer.hpp"

#include <boost/timer/timer.hpp>
#include <boost/chrono.hpp>

using namespace SolAR;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::MODULES::TOOLS;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::api::solver::pose;
namespace xpcf  = org::bcom::xpcf;

#include <string>

#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds




CV_Main::CV_Main()
    : m_camera_ready(false), m_image(nullptr), m_image_reader(nullptr), m_native_camera(nullptr), scan_mode(false) {

  // This issue is because OpenCV takes a file name but NDK AAssetManager only gives file descriptor
  if( !face_cascade.load( face_cascade_name ) ){ LOGE("--(!)Error loading face cascade\n"); };
  if( !eyes_cascade.load( eyes_cascade_name ) ){ LOGE("--(!)Error loading eyes cascade\n"); };
  if( !cars_cascade.load( cars_cascade_name ) ){ LOGE("--(!)Error loading cars cascade\n"); };

};

CV_Main::~CV_Main() {
  // clean up VM and callback handles
  JNIEnv* env;
  java_vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6);
  env->DeleteGlobalRef(calling_activity_obj);
  calling_activity_obj = nullptr;

  // ACameraCaptureSession_stopRepeating(m_capture_session);
  if (m_native_camera != nullptr) {
    delete m_native_camera;
    m_native_camera = nullptr;
  }

  // make sure we don't leak native windows
  if (m_native_window != nullptr) {
    ANativeWindow_release(m_native_window);
    m_native_window = nullptr;
  }

  if (m_image_reader != nullptr) {
    delete (m_image_reader);
    m_image_reader = nullptr;
  }
}

void CV_Main::OnCreate(JNIEnv* env, jobject caller_activity) {
  // Need to create an instance of the Java activity
  calling_activity_obj = env->NewGlobalRef(caller_activity);

  // Need to enter package and class to find Java class
  jclass handler_class = env->GetObjectClass(caller_activity);

  // Create function pointeACameraManager_getCameraCharacteristicsr to use for
  // on_loaded callbacks
  // on_callback = env->GetMethodID(handler_class, "JAVA_FUNCTION", "()V");
}

void CV_Main::OnPause() {}

void CV_Main::OnDestroy() {}

void CV_Main::SetNativeWindow(ANativeWindow* native_window) {
  // Save native window
  m_native_window = native_window;
}

void CV_Main::SetUpCamera() {

  m_native_camera = new NativeCamera(m_selected_camera_type);

  m_native_camera->MatchCaptureSizeRequest(&m_view,
                                           ANativeWindow_getWidth(m_native_window),
                                           ANativeWindow_getHeight(m_native_window));

  ASSERT(m_view.width && m_view.height, "Could not find supportable resolution");
  LOGI("### setup camera - resolution is  : %d %d", m_view.width, m_view.height);

  // Here we set the buffer to use RGBX_8888 as default might be; RGB_565
  ANativeWindow_setBuffersGeometry(m_native_window, m_view.height, m_view.width,
                                   WINDOW_FORMAT_RGBX_8888);

  m_image_reader = new ImageReader(&m_view, AIMAGE_FORMAT_YUV_420_888);
  m_image_reader->SetPresentRotation(m_native_camera->GetOrientation());

  ANativeWindow* image_reader_window = m_image_reader->GetNativeWindow();

  m_camera_ready = m_native_camera->CreateCaptureSession(image_reader_window);
}

void CV_Main::CameraLoop() {
    LOGI("#### Starting camera loop");
    bool buffer_printout = false;


    //////////////////////////////////////////////////////////////////////////////////////////////////
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    if(xpcfComponentManager->load("/data/data/com.example.atadrist.opencvnewsample/config/conf_NaturalImageMarker.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOGE("Failed to load the configuration file conf_NaturalImageMarker.xml");
        return;
    }
    // declare and create components
    LOGI("Start creating components");

    auto camera = xpcfComponentManager->create<SolARCameraOpencv>()->bindTo<input::devices::ICamera>();
    auto imageViewerKeypoints = xpcfComponentManager->create<SolARImageViewerOpencv>("keypoints")->bindTo<display::IImageViewer>();
    auto imageViewerResult = xpcfComponentManager->create<SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();
    auto marker = xpcfComponentManager->create<SolARMarker2DNaturalImageOpencv>()->bindTo<input::files::IMarker2DNaturalImage>();
    auto kpDetector = xpcfComponentManager->create<SolARKeypointDetectorOpencv>()->bindTo<features::IKeypointDetector>();
    auto matcher = xpcfComponentManager->create<SolARDescriptorMatcherKNNOpencv>()->bindTo<features::IDescriptorMatcher>();
    auto basicMatchesFilter = xpcfComponentManager->create<SolARBasicMatchesFilter>()->bindTo<features::IMatchesFilter>();
    auto geomMatchesFilter = xpcfComponentManager->create<SolARGeometricMatchesFilterOpencv>()->bindTo<features::IMatchesFilter>();
    auto homographyEstimation = xpcfComponentManager->create<SolARHomographyEstimationOpencv>()->bindTo<solver::pose::I2DTransformFinder>();
    auto homographyValidation = xpcfComponentManager->create<SolARHomographyValidation>()->bindTo<solver::pose::IHomographyValidation>();
    auto keypointsReindexer = xpcfComponentManager->create<SolARKeypointsReIndexer>()->bindTo<features::IKeypointsReIndexer>();
    auto poseEstimation = xpcfComponentManager->create<SolARPoseEstimationPnpOpencv>()->bindTo<solver::pose::I3DTransformFinderFrom2D3D>();
    //auto poseEstimation =xpcfComponentManager->create<SolARPoseEstimationPnpEPFL>()->bindTo<solver::pose::I3DTransformFinderFrom2D3D>();
    auto overlay2DComponent = xpcfComponentManager->create<SolAR2DOverlayOpencv>()->bindTo<display::I2DOverlay>();
    auto overlay3DComponent = xpcfComponentManager->create<SolAR3DOverlayBoxOpencv>("nat")->bindTo<display::I3DOverlay>();
    auto img_mapper = xpcfComponentManager->create<SolARImage2WorldMapper4Marker2D>()->bindTo<geom::IImage2WorldMapper>();
    auto transform2D = xpcfComponentManager->create<SolAR2DTransform>()->bindTo<geom::I2DTransform>();
    auto descriptorExtractor =  xpcfComponentManager->create<SolARDescriptorsExtractorAKAZE2Opencv>()->bindTo<features::IDescriptorsExtractor>();


    /* in dynamic mode, we need to check that components are well created*/
    /* this is needed in dynamic mode */
    if (!imageViewerKeypoints || !imageViewerResult || !marker || !kpDetector || !descriptorExtractor || !matcher || !basicMatchesFilter || !geomMatchesFilter || !homographyEstimation ||
        !homographyValidation ||!keypointsReindexer || !poseEstimation || !overlay2DComponent || !overlay3DComponent || !img_mapper || !transform2D ) {
        LOGE("One or more component creations have failed");
        return;
    }
    LOGI("All components have been created");

    // the following code is common to the 3 samples (simple, compile-time, run-time)
    //
    //
    // Declare data structures used to exchange information between components
    SRef<Image> refImage,   camImage, kpImageCam;
    SRef<DescriptorBuffer> refDescriptors, camDescriptors;
    std::vector<DescriptorMatch> matches;

    Transform2Df Hm;
    std::vector< SRef<Keypoint> > refKeypoints, camKeypoints;  // where to store detected keypoints in ref image and camera image

    // load marker
    LOGI("LOAD MARKER IMAGE ");
    marker->loadMarker();

    LOGI("GET MARKER IMAGE ");
    marker->getImage(refImage);

    // NOT WORKING ! Set the size of the box to the size of the natural image marker
    overlay3DComponent->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(marker->getWidth(),0);
    overlay3DComponent->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(marker->getHeight(),1);
    overlay3DComponent->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(marker->getHeight()/2.0f,2);

    // detect keypoints in reference image
    LOGI("DETECT MARKER KEYPOINTS ");
    kpDetector->detect(refImage, refKeypoints);

    // extract descriptors in reference image
    LOGI("EXTRACT MARKER DESCRIPTORS ");
    descriptorExtractor->extract(refImage, refKeypoints, refDescriptors);
    LOGI("EXTRACT MARKER DESCRIPTORS COMPUTED");

    // initialize overlay 3D component with the camera intrinsec parameters (please refeer to the use of intrinsec parameters file)
    overlay3DComponent->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());

    // initialize pose estimation
    poseEstimation->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());

    // initialize image mapper with the reference image size and marker size
    img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalWidth")->setIntegerValue(refImage->getSize().width);
    img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalHeight")->setIntegerValue(refImage->getSize().height);
    img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("worldWidth")->setFloatingValue(marker->getSize().width);
    img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("worldHeight")->setFloatingValue(marker->getSize().height);

    // to count the average number of processed frames per seconds
    boost::timer::cpu_timer mytimer;
    clock_t start, end;
    int count = 0;
    start = clock();

    // vector of 4 corners in the marker
    std::vector<SRef <Point2Df>> refImgCorners;
    Point2Df corner0(0, 0);
    Point2Df corner1((float)refImage->getWidth(), 0);
    Point2Df corner2((float)refImage->getWidth(), (float)refImage->getHeight());
    Point2Df corner3(0, (float)refImage->getHeight());
    refImgCorners.push_back(xpcf::utils::make_shared<Point2Df>(corner0));
    refImgCorners.push_back(xpcf::utils::make_shared<Point2Df>(corner1));
    refImgCorners.push_back(xpcf::utils::make_shared<Point2Df>(corner2));
    refImgCorners.push_back(xpcf::utils::make_shared<Point2Df>(corner3));
    //////////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Fiducial components
    SRef<xpcf::IComponentManager> xpcfComponentManagerFid = xpcf::getComponentManagerInstance();
    if(xpcfComponentManagerFid->load("/data/data/com.example.atadrist.opencvnewsample/config/conf_FiducialMarker.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOGE("Failed to load the configuration file conf_FiducialMarker.xml");
        return;
    }
    // declare and create components
    LOGI("Start creating components");
    //auto camera = xpcfComponentManagerFid->create<SolARCameraOpencv>()->bindTo<input::devices::ICamera>();

    auto binaryMarker =xpcfComponentManager->create<SolARMarker2DSquaredBinaryOpencv>()->bindTo<input::files::IMarker2DSquaredBinary>();

    auto imageViewer =xpcfComponentManager->create<SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();

    auto imageFilterBinary =xpcfComponentManager->create<SolARImageFilterBinaryOpencv>()->bindTo<image::IImageFilter>();
    auto imageConvertor =xpcfComponentManager->create<SolARImageConvertorOpencv>()->bindTo<image::IImageConvertor>();
    auto contoursExtractor =xpcfComponentManager->create<SolARContoursExtractorOpencv>()->bindTo<features::IContoursExtractor>();
    auto contoursFilter =xpcfComponentManager->create<SolARContoursFilterBinaryMarkerOpencv>()->bindTo<features::IContoursFilter>();
    auto perspectiveController =xpcfComponentManager->create<SolARPerspectiveControllerOpencv>()->bindTo<image::IPerspectiveController>();
    auto patternDescriptorExtractor =xpcfComponentManager->create<SolARDescriptorsExtractorSBPatternOpencv>()->bindTo<features::IDescriptorsExtractorSBPattern>();

    auto patternMatcher =xpcfComponentManager->create<SolARDescriptorMatcherRadiusOpencv>()->bindTo<features::IDescriptorMatcher>();
    auto patternReIndexer = xpcfComponentManager->create<SolARSBPatternReIndexer>()->bindTo<features::ISBPatternReIndexer>();

    auto img2worldMapper = xpcfComponentManager->create<SolARImage2WorldMapper4Marker2D>()->bindTo<geom::IImage2WorldMapper>();
    auto PnP =xpcfComponentManager->create<SolARPoseEstimationPnpOpencv>()->bindTo<solver::pose::I3DTransformFinderFrom2D3D>();
    auto overlay3D =xpcfComponentManager->create<SolAR3DOverlayBoxOpencv>("fid")->bindTo<display::I3DOverlay>();
    auto overlay2DContours =xpcfComponentManager->create<SolAR2DOverlayOpencv>("contours")->bindTo<display::I2DOverlay>();
    auto overlay2DCircles =xpcfComponentManager->create<SolAR2DOverlayOpencv>("circles")->bindTo<display::I2DOverlay>();


    SRef<Image> inputImage;
    SRef<Image> greyImage;
    SRef<Image> binaryImage;
    SRef<Image> contoursImage;
    SRef<Image> filteredContoursImage;

    std::vector<SRef<Contour2Df>>              contours;
    std::vector<SRef<Contour2Df>>              filtered_contours;
    std::vector<SRef<Image>>                   patches;
    std::vector<SRef<Contour2Df>>              recognizedContours;
    SRef<DescriptorBuffer>                     recognizedPatternsDescriptors;
    SRef<DescriptorBuffer>                     markerPatternDescriptor;
    std::vector<DescriptorMatch>               patternMatches;
    std::vector<SRef<Point2Df>>                pattern2DPoints;
    std::vector<SRef<Point2Df>>                img2DPoints;
    std::vector<SRef<Point3Df>>                pattern3DPoints;
    Transform3Df                               pose;

    CamCalibration K;

    // components initialisation
    binaryMarker->loadMarker();
    patternDescriptorExtractor->extract(binaryMarker->getPattern(), markerPatternDescriptor);

    LOG_DEBUG ("Marker pattern:\n {}", binaryMarker->getPattern()->getPatternMatrix())

    // Set the size of the box to display according to the marker size in world unit
    overlay3D->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(binaryMarker->getSize().width,0);
    overlay3D->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(binaryMarker->getSize().height,1);
    overlay3D->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(binaryMarker->getSize().height/2.0f,2);


    int patternSize = binaryMarker->getPattern()->getSize();

    patternDescriptorExtractor->bindTo<xpcf::IConfigurable>()->getProperty("patternSize")->setIntegerValue(patternSize);
    patternReIndexer->bindTo<xpcf::IConfigurable>()->getProperty("sbPatternSize")->setIntegerValue(patternSize);

    // NOT WORKING ! initialize image mapper with the reference image size and marker size
    img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalWidth")->setIntegerValue(patternSize);
    img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalHeight")->setIntegerValue(patternSize);
    img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("worldWidth")->setFloatingValue(binaryMarker->getSize().width);
    img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("worldHeight")->setFloatingValue(binaryMarker->getSize().height);

    PnP->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
    overlay3D->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());

    // to count the average number of processed frames per seconds
    count=0;

    start = clock();

    //cv::Mat img_temp;
    bool process = true;
    //////////////////////////////////////////////////////////////////////////////////////////////////

    LOGI("CAMERA LOOP");
    while (1) {
        ///// android specific code to get camera frame
        if (m_camera_thread_stopped) { break; }
        if (!m_camera_ready || !m_image_reader) { continue; }
        m_image = m_image_reader->GetLatestImage();
        if (m_image == nullptr) { continue; }

        ANativeWindow_acquire(m_native_window);
        ANativeWindow_Buffer buffer;
        if (ANativeWindow_lock(m_native_window, &buffer, nullptr) < 0) {
            m_image_reader->DeleteImage(m_image);
            m_image = nullptr;
            continue;
        }

        if (false == buffer_printout) {
            buffer_printout = true;
            LOGI("/// H-W-S-F: %d, %d, %d, %d", buffer.height, buffer.width, buffer.stride, buffer.format);
        }

        m_image_reader->DisplayImage(&buffer, m_image);

        display_mat = cv::Mat(buffer.height, buffer.stride, CV_8UC4, buffer.bits);
        cv::Mat display_mat3;
        cv::cvtColor( display_mat,display_mat3, CV_BGRA2BGR );
        /////////////////////////////////


        if (true == scan_mode) {
            LOGI("KP MODE");
            KPDetect(display_mat);
        }
        if (true == face_mode) {
            LOGI("FACE DETECT MODE");
            FaceDetect(display_mat);
        }

        if (true == nat_mode) {
            LOGI("NATURAL MARKER MODE");
            /////////////////////////////////////////////////////////////////////////////////
            // create a solar image from camera frame
            LOGI("create a solar image from camera frame");
            SolAR::MODULES::OPENCV::SolAROpenCVHelper::convertToSolar(display_mat3,camImage);

            // SolAR::MODULES::OPENCV::SolAROpenCVHelper::convertToSolar(myCam,camImage);
            count++;
            Transform3Df pose;
            // detect keypoints in camera image
            LOGI("detect keypoints in camera image");
            kpDetector->detect(camImage, camKeypoints);
            /* extract descriptors in camera image*/
            LOGI("extract descriptors in camera image");
            descriptorExtractor->extract(camImage, camKeypoints, camDescriptors);
            /*compute matches between reference image and camera image*/
            LOGI("compute matches between reference image and camera image");
            matcher->match(refDescriptors, camDescriptors, matches);
            /* filter matches to remove redundancy and check geometric validity */
            LOGI("filter matches to remove redundancy and check geometric validity");
            basicMatchesFilter->filter(matches, matches, refKeypoints, camKeypoints);
            geomMatchesFilter->filter(matches, matches, refKeypoints, camKeypoints);
            /* we declare here the Solar datastucture we will need for homography*/
            std::vector <SRef<Point2Df>> ref2Dpoints;
            std::vector <SRef<Point2Df>> cam2Dpoints;
            Point2Df point;
            std::vector <SRef<Point3Df>> ref3Dpoints;
            std::vector <SRef<Point2Df>> output2Dpoints;
            std::vector <SRef<Point2Df>> markerCornersinCamImage;
            std::vector <SRef<Point3Df>> markerCornersinWorld;
            /*we consider that, if we have less than 10 matches (arbitrarily), we can't compute homography for the current frame */
            if (matches.size()> 10) {
                // reindex the keypoints with established correspondence after the matching
                keypointsReindexer->reindex(refKeypoints, camKeypoints, matches, ref2Dpoints, cam2Dpoints);

                // mapping to 3D points
                img_mapper->map(ref2Dpoints, ref3Dpoints);

                Transform2DFinder::RetCode res = homographyEstimation->find(ref2Dpoints, cam2Dpoints, Hm);
                //test if a meaningful matrix has been obtained
                if (res == Transform2DFinder::RetCode::TRANSFORM2D_ESTIMATION_OK)
                {
                    // vector of 2D corners in camera image
                    transform2D->transform(refImgCorners, Hm, markerCornersinCamImage);
                    /* we verify is the estimated homography is valid*/
                    if (homographyValidation->isValid(refImgCorners, markerCornersinCamImage))
                    {
                        // from the homography we create 4 points at the corners of the reference image
                        // map corners in 3D world coordinates
                        img_mapper->map(refImgCorners, markerCornersinWorld);

                        // pose from solvePNP using 4 points.
                        /* The pose could also be estimated from all the points used to estimate the homography */
                        poseEstimation->estimate(markerCornersinCamImage, markerCornersinWorld, pose);

                        /* The pose last parameter can not be 0, so this is an error case*/
                        if (pose(3, 3) != 0.0)
                        {
                            /* We draw a box on the place of the recognized natural marker*/
                            overlay3DComponent->draw(pose, camImage);
                            cv::Mat display_mat4;
                            SolAR::MODULES::OPENCV::SolAROpenCVHelper::mapToOpenCV(camImage,display_mat4);
                            cv::cvtColor( display_mat4,display_mat, CV_BGR2BGRA );

                        }
                        else
                        {
                            /* The pose estimated is false: error case*/
                            LOG_INFO("no pose detected for this frame");
                        }
                    }
                    else /* when homography is not valid*/
                        LOG_INFO("Wrong homography for this frame");

                }
            }
            /////////////////////////////////////////////////////////////////////////////////
        }

        if (true == fid_mode) {
            LOGI("FIDUCIAL MARKER MODE");
            //////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////
            // create a solar image from camera frame
            LOGI("create a solar image from camera frame");
            SolAR::MODULES::OPENCV::SolAROpenCVHelper::convertToSolar(display_mat3,inputImage);
            // Convert Image from RGB to grey
            imageConvertor->convert(inputImage, greyImage, Image::ImageLayout::LAYOUT_GREY);

            // Convert Image from grey to black and white
            imageFilterBinary->filter(greyImage,binaryImage);

            // Extract contours from binary image
            contoursExtractor->extract(binaryImage,contours);
            // Filter 4 edges contours to find those candidate for marker contours
            contoursFilter->filter(contours, filtered_contours);


            // Create one warpped and cropped image by contour
            perspectiveController->correct(binaryImage, filtered_contours, patches);

            // test if this last image is really a squared binary marker, and if it is the case, extract its descriptor
            if (patternDescriptorExtractor->extract(patches, filtered_contours, recognizedPatternsDescriptors, recognizedContours) != FrameworkReturnCode::_ERROR_)
            {



                // From extracted squared binary pattern, match the one corresponding to the squared binary marker
                if (patternMatcher->match(markerPatternDescriptor, recognizedPatternsDescriptors, patternMatches) == features::DescriptorMatcher::DESCRIPTORS_MATCHER_OK)
                {


                    // Reindex the pattern to create two vector of points, the first one corresponding to marker corner, the second one corresponding to the poitsn of the contour
                    patternReIndexer->reindex(recognizedContours, patternMatches, pattern2DPoints, img2DPoints);
                    // Compute the 3D position of each corner of the marker
                    img2worldMapper->map(pattern2DPoints, pattern3DPoints);

                    // Compute the pose of the camera using a Perspective n Points algorithm using only the 4 corners of the marker
                    if (PnP->estimate(img2DPoints, pattern3DPoints, pose) == FrameworkReturnCode::_SUCCESS)
                    {
                        LOG_DEBUG("Camera pose : \n {}", pose.matrix());
                        // Display a 3D box over the marker
                        overlay3D->draw(pose,inputImage);
                        cv::Mat display_mat4;
                        SolAR::MODULES::OPENCV::SolAROpenCVHelper::mapToOpenCV(inputImage,display_mat4);
                        cv::cvtColor( display_mat4,display_mat, CV_BGR2BGRA );
                    }
                }
            }

            //////////////////////////////////////////////////////////////////////////////////
        }
        ANativeWindow_unlockAndPost(m_native_window);
        ANativeWindow_release(m_native_window);
    }
    FlipCamera();
}

void CV_Main::SolARCameraLoop_full() {
    LOGI("#### Starting camera loop");
    bool buffer_printout = false;


    //////////////////////////////////////////////////////////////////////////////////////////////////
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    if(xpcfComponentManager->load("/data/data/com.example.atadrist.opencvnewsample/config/conf_NaturalImageMarker.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOGE("Failed to load the configuration file conf_NaturalImageMarker.xml");
        return;
    }
    // declare and create components
    LOGI("Start creating components");

    auto camera = xpcfComponentManager->create<SolARCameraOpencv>()->bindTo<input::devices::ICamera>();
    auto imageViewerKeypoints = xpcfComponentManager->create<SolARImageViewerOpencv>("keypoints")->bindTo<display::IImageViewer>();
    auto imageViewerResult = xpcfComponentManager->create<SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();
    auto marker = xpcfComponentManager->create<SolARMarker2DNaturalImageOpencv>()->bindTo<input::files::IMarker2DNaturalImage>();
    auto kpDetector = xpcfComponentManager->create<SolARKeypointDetectorOpencv>()->bindTo<features::IKeypointDetector>();
    auto matcher = xpcfComponentManager->create<SolARDescriptorMatcherKNNOpencv>()->bindTo<features::IDescriptorMatcher>();
    auto basicMatchesFilter = xpcfComponentManager->create<SolARBasicMatchesFilter>()->bindTo<features::IMatchesFilter>();
    auto geomMatchesFilter = xpcfComponentManager->create<SolARGeometricMatchesFilterOpencv>()->bindTo<features::IMatchesFilter>();
    auto homographyEstimation = xpcfComponentManager->create<SolARHomographyEstimationOpencv>()->bindTo<solver::pose::I2DTransformFinder>();
    auto homographyValidation = xpcfComponentManager->create<SolARHomographyValidation>()->bindTo<solver::pose::IHomographyValidation>();
    auto keypointsReindexer = xpcfComponentManager->create<SolARKeypointsReIndexer>()->bindTo<features::IKeypointsReIndexer>();
    auto poseEstimation = xpcfComponentManager->create<SolARPoseEstimationPnpOpencv>()->bindTo<solver::pose::I3DTransformFinderFrom2D3D>();
    //auto poseEstimation =xpcfComponentManager->create<SolARPoseEstimationPnpEPFL>()->bindTo<solver::pose::I3DTransformFinderFrom2D3D>();
    auto overlay2DComponent = xpcfComponentManager->create<SolAR2DOverlayOpencv>()->bindTo<display::I2DOverlay>();
    auto overlay3DComponent = xpcfComponentManager->create<SolAR3DOverlayBoxOpencv>()->bindTo<display::I3DOverlay>();
    auto img_mapper = xpcfComponentManager->create<SolARImage2WorldMapper4Marker2D>()->bindTo<geom::IImage2WorldMapper>();
    auto transform2D = xpcfComponentManager->create<SolAR2DTransform>()->bindTo<geom::I2DTransform>();
    auto descriptorExtractor =  xpcfComponentManager->create<SolARDescriptorsExtractorAKAZE2Opencv>()->bindTo<features::IDescriptorsExtractor>();


    /* in dynamic mode, we need to check that components are well created*/
    /* this is needed in dynamic mode */
    if (!imageViewerKeypoints || !imageViewerResult || !marker || !kpDetector || !descriptorExtractor || !matcher || !basicMatchesFilter || !geomMatchesFilter || !homographyEstimation ||
        !homographyValidation ||!keypointsReindexer || !poseEstimation || !overlay2DComponent || !overlay3DComponent || !img_mapper || !transform2D ) {
        LOGE("One or more component creations have failed");
        return;
    }
    LOGI("All components have been created");

    // the following code is common to the 3 samples (simple, compile-time, run-time)
    //
    //
    // Declare data structures used to exchange information between components
    SRef<Image> refImage,   camImage, kpImageCam;
    SRef<DescriptorBuffer> refDescriptors, camDescriptors;
    std::vector<DescriptorMatch> matches;

    Transform2Df Hm;
    std::vector< SRef<Keypoint> > refKeypoints, camKeypoints;  // where to store detected keypoints in ref image and camera image

    // load marker
    LOGI("LOAD MARKER IMAGE ");
    marker->loadMarker();

    LOGI("GET MARKER IMAGE ");
    marker->getImage(refImage);

    // NOT WORKING ! Set the size of the box to the size of the natural image marker
    overlay3DComponent->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(marker->getWidth(),0);
    overlay3DComponent->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(marker->getHeight(),1);
    overlay3DComponent->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(marker->getHeight()/2.0f,2);

    // detect keypoints in reference image
    LOGI("DETECT MARKER KEYPOINTS ");
    kpDetector->detect(refImage, refKeypoints);

    // extract descriptors in reference image
    LOGI("EXTRACT MARKER DESCRIPTORS ");
    descriptorExtractor->extract(refImage, refKeypoints, refDescriptors);
    LOGI("EXTRACT MARKER DESCRIPTORS COMPUTED");

    // initialize overlay 3D component with the camera intrinsec parameters (please refeer to the use of intrinsec parameters file)
    overlay3DComponent->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());

    // initialize pose estimation
    poseEstimation->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());

    // initialize image mapper with the reference image size and marker size
    img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalWidth")->setIntegerValue(refImage->getSize().width);
    img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalHeight")->setIntegerValue(refImage->getSize().height);
    img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("worldWidth")->setFloatingValue(marker->getSize().width);
    img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("worldHeight")->setFloatingValue(marker->getSize().height);

    // to count the average number of processed frames per seconds
    boost::timer::cpu_timer mytimer;
    clock_t start, end;
    int count = 0;
    start = clock();

    // vector of 4 corners in the marker
    std::vector<SRef <Point2Df>> refImgCorners;
    Point2Df corner0(0, 0);
    Point2Df corner1((float)refImage->getWidth(), 0);
    Point2Df corner2((float)refImage->getWidth(), (float)refImage->getHeight());
    Point2Df corner3(0, (float)refImage->getHeight());
    refImgCorners.push_back(xpcf::utils::make_shared<Point2Df>(corner0));
    refImgCorners.push_back(xpcf::utils::make_shared<Point2Df>(corner1));
    refImgCorners.push_back(xpcf::utils::make_shared<Point2Df>(corner2));
    refImgCorners.push_back(xpcf::utils::make_shared<Point2Df>(corner3));
    //////////////////////////////////////////////////////////////////////////////////////////////////

    LOGI("CAMERA LOOP");
    while (1) {
        if (m_camera_thread_stopped) { break; }
        if (!m_camera_ready || !m_image_reader) { continue; }
        m_image = m_image_reader->GetLatestImage();
        if (m_image == nullptr) { continue; }

        ANativeWindow_acquire(m_native_window);
        ANativeWindow_Buffer buffer;
        if (ANativeWindow_lock(m_native_window, &buffer, nullptr) < 0) {
            m_image_reader->DeleteImage(m_image);
            m_image = nullptr;
            continue;
        }

        if (false == buffer_printout) {
            buffer_printout = true;
            LOGI("/// H-W-S-F: %d, %d, %d, %d", buffer.height, buffer.width, buffer.stride, buffer.format);
        }

        m_image_reader->DisplayImage(&buffer, m_image);

        display_mat = cv::Mat(buffer.height, buffer.stride, CV_8UC4, buffer.bits);
        cv::Mat display_mat3;
        cv::cvtColor( display_mat,display_mat3, CV_BGRA2BGR );


        //cv::flip(display_mat,display_mat,1);

        if (true == scan_mode) {
            //FaceDetect(display_mat);
            //KPDetect(display_mat);

            /////////////////////////////////////////////////////////////////////////////////
            // create a solar image from camera frame
            LOGI("create a solar image from camera frame");
            SolAR::MODULES::OPENCV::SolAROpenCVHelper::convertToSolar(display_mat3,camImage);

            // SolAR::MODULES::OPENCV::SolAROpenCVHelper::convertToSolar(myCam,camImage);
            count++;
            Transform3Df pose;
            // detect keypoints in camera image
            LOGI("detect keypoints in camera image");
            kpDetector->detect(camImage, camKeypoints);
            /* extract descriptors in camera image*/
            LOGI("extract descriptors in camera image");
            descriptorExtractor->extract(camImage, camKeypoints, camDescriptors);
            /*compute matches between reference image and camera image*/
            LOGI("compute matches between reference image and camera image");
            matcher->match(refDescriptors, camDescriptors, matches);
            /* filter matches to remove redundancy and check geometric validity */
            LOGI("filter matches to remove redundancy and check geometric validity");
            basicMatchesFilter->filter(matches, matches, refKeypoints, camKeypoints);
            geomMatchesFilter->filter(matches, matches, refKeypoints, camKeypoints);
            /* we declare here the Solar datastucture we will need for homography*/
            std::vector <SRef<Point2Df>> ref2Dpoints;
            std::vector <SRef<Point2Df>> cam2Dpoints;
            Point2Df point;
            std::vector <SRef<Point3Df>> ref3Dpoints;
            std::vector <SRef<Point2Df>> output2Dpoints;
            std::vector <SRef<Point2Df>> markerCornersinCamImage;
            std::vector <SRef<Point3Df>> markerCornersinWorld;
            /*we consider that, if we have less than 10 matches (arbitrarily), we can't compute homography for the current frame */
            if (matches.size()> 10) {
                // reindex the keypoints with established correspondence after the matching
                keypointsReindexer->reindex(refKeypoints, camKeypoints, matches, ref2Dpoints, cam2Dpoints);

                // mapping to 3D points
                img_mapper->map(ref2Dpoints, ref3Dpoints);

                Transform2DFinder::RetCode res = homographyEstimation->find(ref2Dpoints, cam2Dpoints, Hm);
                //test if a meaningful matrix has been obtained
                if (res == Transform2DFinder::RetCode::TRANSFORM2D_ESTIMATION_OK)
                {
                    //poseEstimation->poseFromHomography(Hm,pose,objectCorners,sceneCorners);
                    // vector of 2D corners in camera image
                    transform2D->transform(refImgCorners, Hm, markerCornersinCamImage);
                    // draw circles on corners in camera image
                    //overlay2DComponent->drawCircles(markerCornersinCamImage, 10, 5, kpImageCam);

                    /* we verify is the estimated homography is valid*/
                    if (homographyValidation->isValid(refImgCorners, markerCornersinCamImage))
                    {
                        // from the homography we create 4 points at the corners of the reference image
                        // map corners in 3D world coordinates
                        img_mapper->map(refImgCorners, markerCornersinWorld);

                        // pose from solvePNP using 4 points.
                        /* The pose could also be estimated from all the points used to estimate the homography */
                        poseEstimation->estimate(markerCornersinCamImage, markerCornersinWorld, pose);

                        /* The pose last parameter can not be 0, so this is an error case*/
                        if (pose(3, 3) != 0.0)
                        {
                            /* We draw a box on the place of the recognized natural marker*/
                            overlay3DComponent->draw(pose, camImage);
                            cv::Mat display_mat4;
                            SolAR::MODULES::OPENCV::SolAROpenCVHelper::mapToOpenCV(camImage,display_mat4);
                            cv::cvtColor( display_mat4,display_mat, CV_BGR2BGRA );

                        }
                        else
                        {
                            /* The pose estimated is false: error case*/
                            LOG_INFO("no pose detected for this frame");
                        }
                    }
                    else /* when homography is not valid*/
                    LOG_INFO("Wrong homography for this frame");

                }
            }
            /////////////////////////////////////////////////////////////////////////////////
        }

        ANativeWindow_unlockAndPost(m_native_window);
        ANativeWindow_release(m_native_window);
    }
    FlipCamera();
}

void CV_Main::SolARCameraLoop() {
    LOGI("#### Starting camera loop");
    bool buffer_printout = false;


    //////////////////////////////////////////////////////////////////////////////////////////////////
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    if(xpcfComponentManager->load("/data/data/com.example.atadrist.opencvnewsample/config/conf_FiducialMarker.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOGE("Failed to load the configuration file conf_FiducialMarker.xml");
        return;
    }
    // declare and create components
    LOGI("Start creating components");
    auto camera = xpcfComponentManager->create<SolARCameraOpencv>()->bindTo<input::devices::ICamera>();

    auto binaryMarker =xpcfComponentManager->create<SolARMarker2DSquaredBinaryOpencv>()->bindTo<input::files::IMarker2DSquaredBinary>();

    auto imageViewer =xpcfComponentManager->create<SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();
    auto imageViewerGrey =xpcfComponentManager->create<SolARImageViewerOpencv>("grey")->bindTo<display::IImageViewer>();
    auto imageViewerBinary =xpcfComponentManager->create<SolARImageViewerOpencv>("binary")->bindTo<display::IImageViewer>();
    auto imageViewerContours =xpcfComponentManager->create<SolARImageViewerOpencv>("contours")->bindTo<display::IImageViewer>();
    auto imageViewerFilteredContours =xpcfComponentManager->create<SolARImageViewerOpencv>("filteredContours")->bindTo<display::IImageViewer>();

    auto imageFilterBinary =xpcfComponentManager->create<SolARImageFilterBinaryOpencv>()->bindTo<image::IImageFilter>();
    auto imageConvertor =xpcfComponentManager->create<SolARImageConvertorOpencv>()->bindTo<image::IImageConvertor>();
    auto contoursExtractor =xpcfComponentManager->create<SolARContoursExtractorOpencv>()->bindTo<features::IContoursExtractor>();
    auto contoursFilter =xpcfComponentManager->create<SolARContoursFilterBinaryMarkerOpencv>()->bindTo<features::IContoursFilter>();
    auto perspectiveController =xpcfComponentManager->create<SolARPerspectiveControllerOpencv>()->bindTo<image::IPerspectiveController>();
    auto patternDescriptorExtractor =xpcfComponentManager->create<SolARDescriptorsExtractorSBPatternOpencv>()->bindTo<features::IDescriptorsExtractorSBPattern>();

    auto patternMatcher =xpcfComponentManager->create<SolARDescriptorMatcherRadiusOpencv>()->bindTo<features::IDescriptorMatcher>();
    auto patternReIndexer = xpcfComponentManager->create<SolARSBPatternReIndexer>()->bindTo<features::ISBPatternReIndexer>();

    auto img2worldMapper = xpcfComponentManager->create<SolARImage2WorldMapper4Marker2D>()->bindTo<geom::IImage2WorldMapper>();
    auto PnP =xpcfComponentManager->create<SolARPoseEstimationPnpOpencv>()->bindTo<solver::pose::I3DTransformFinderFrom2D3D>();
    auto overlay3D =xpcfComponentManager->create<SolAR3DOverlayBoxOpencv>()->bindTo<display::I3DOverlay>();
    auto overlay2DContours =xpcfComponentManager->create<SolAR2DOverlayOpencv>("contours")->bindTo<display::I2DOverlay>();
    auto overlay2DCircles =xpcfComponentManager->create<SolAR2DOverlayOpencv>("circles")->bindTo<display::I2DOverlay>();


    SRef<Image> inputImage;
    SRef<Image> greyImage;
    SRef<Image> binaryImage;
    SRef<Image> contoursImage;
    SRef<Image> filteredContoursImage;

    std::vector<SRef<Contour2Df>>              contours;
    std::vector<SRef<Contour2Df>>              filtered_contours;
    std::vector<SRef<Image>>                   patches;
    std::vector<SRef<Contour2Df>>              recognizedContours;
    SRef<DescriptorBuffer>                     recognizedPatternsDescriptors;
    SRef<DescriptorBuffer>                     markerPatternDescriptor;
    std::vector<DescriptorMatch>               patternMatches;
    std::vector<SRef<Point2Df>>                pattern2DPoints;
    std::vector<SRef<Point2Df>>                img2DPoints;
    std::vector<SRef<Point3Df>>                pattern3DPoints;
    Transform3Df                               pose;

    CamCalibration K;

    // components initialisation
    binaryMarker->loadMarker();
    patternDescriptorExtractor->extract(binaryMarker->getPattern(), markerPatternDescriptor);

    LOG_DEBUG ("Marker pattern:\n {}", binaryMarker->getPattern()->getPatternMatrix())

    // Set the size of the box to display according to the marker size in world unit
    overlay3D->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(binaryMarker->getSize().width,0);
    overlay3D->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(binaryMarker->getSize().height,1);
    overlay3D->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(binaryMarker->getSize().height/2.0f,2);


    int patternSize = binaryMarker->getPattern()->getSize();

    patternDescriptorExtractor->bindTo<xpcf::IConfigurable>()->getProperty("patternSize")->setIntegerValue(patternSize);
    patternReIndexer->bindTo<xpcf::IConfigurable>()->getProperty("sbPatternSize")->setIntegerValue(patternSize);

    // NOT WORKING ! initialize image mapper with the reference image size and marker size
    img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalWidth")->setIntegerValue(patternSize);
    img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalHeight")->setIntegerValue(patternSize);
    img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("worldWidth")->setFloatingValue(binaryMarker->getSize().width);
    img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("worldHeight")->setFloatingValue(binaryMarker->getSize().height);

    PnP->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
    overlay3D->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());

    // to count the average number of processed frames per seconds
    int count=0;
    clock_t start,end;
    start= clock();

    //cv::Mat img_temp;
    bool process = true;
    //////////////////////////////////////////////////////////////////////////////////////////////////

    LOGI("CAMERA LOOP");
    while (1) {
        if (m_camera_thread_stopped) { break; }
        if (!m_camera_ready || !m_image_reader) { continue; }
        m_image = m_image_reader->GetLatestImage();
        if (m_image == nullptr) { continue; }

        ANativeWindow_acquire(m_native_window);
        ANativeWindow_Buffer buffer;
        if (ANativeWindow_lock(m_native_window, &buffer, nullptr) < 0) {
            m_image_reader->DeleteImage(m_image);
            m_image = nullptr;
            continue;
        }

        if (false == buffer_printout) {
            buffer_printout = true;
            LOGI("/// H-W-S-F: %d, %d, %d, %d", buffer.height, buffer.width, buffer.stride, buffer.format);
        }

        m_image_reader->DisplayImage(&buffer, m_image);

        display_mat = cv::Mat(buffer.height, buffer.stride, CV_8UC4, buffer.bits);
        cv::Mat display_mat3;
        cv::cvtColor( display_mat,display_mat3, CV_BGRA2BGR );


        //cv::flip(display_mat,display_mat,1);

        if (true == scan_mode) {
            //FaceDetect(display_mat);
            //KPDetect(display_mat);
            /////////////////////////////////////////////////////////////////////////////////
            // create a solar image from camera frame
            LOGI("create a solar image from camera frame");
            SolAR::MODULES::OPENCV::SolAROpenCVHelper::convertToSolar(display_mat3,inputImage);
            // Convert Image from RGB to grey
            imageConvertor->convert(inputImage, greyImage, Image::ImageLayout::LAYOUT_GREY);

            // Convert Image from grey to black and white
            imageFilterBinary->filter(greyImage,binaryImage);

            // Extract contours from binary image
            contoursExtractor->extract(binaryImage,contours);
#ifndef NDEBUG
            contoursImage = binaryImage->copy();
            overlay2DContours->drawContours(contours, contoursImage);
#endif
            // Filter 4 edges contours to find those candidate for marker contours
            contoursFilter->filter(contours, filtered_contours);

#ifndef NDEBUG
            filteredContoursImage = binaryImage->copy();
            overlay2DContours->drawContours(filtered_contours, filteredContoursImage);
#endif
            // Create one warpped and cropped image by contour
            perspectiveController->correct(binaryImage, filtered_contours, patches);

            // test if this last image is really a squared binary marker, and if it is the case, extract its descriptor
            if (patternDescriptorExtractor->extract(patches, filtered_contours, recognizedPatternsDescriptors, recognizedContours) != FrameworkReturnCode::_ERROR_)
            {

#ifndef NDEBUG
                LOG_DEBUG("Looking for the following descriptor:");
                for (uint32_t i = 0; i < markerPatternDescriptor->getNbDescriptors()*markerPatternDescriptor->getDescriptorByteSize(); i++)
                {

                    if (i%patternSize == 0)
                        std::cout<<"[";
                    if (i%patternSize != patternSize-1)
                        std::cout << (int)((unsigned char*)markerPatternDescriptor->data())[i] << ", ";
                    else
                        std::cout << (int)((unsigned char*)markerPatternDescriptor->data())[i] <<"]" << std::endl;
                }
                std::cout << std::endl;

                std::cout << recognizedPatternsDescriptors->getNbDescriptors() <<" recognized Pattern Descriptors " << std::endl;
                int desrciptorSize = recognizedPatternsDescriptors->getDescriptorByteSize();
                for (uint32_t i = 0; i < recognizedPatternsDescriptors->getNbDescriptors()/4; i++)
                {
                    for (int j = 0; j < patternSize; j++)
                    {
                        for (int k = 0; k < 4; k++)
                        {
                            std::cout<<"[";
                            for (int l = 0; l < patternSize; l++)
                            {
                                std::cout << (int)((unsigned char*)recognizedPatternsDescriptors->data())[desrciptorSize*((i*4)+k) + j*patternSize + l];
                                if (l != patternSize-1)
                                    std::cout << ", ";
                            }
                            std::cout <<"]";
                        }
                        std::cout << std::endl;
                    }
                    std::cout << std::endl << std::endl;
                }

                std::cout << recognizedContours.size() <<" Recognized Pattern contour " << std::endl;
                for (int i = 0; i < recognizedContours.size()/4; i++)
                {
                    for (int j = 0; j < recognizedContours[0]->size(); j++)
                    {
                        for (int k = 0; k < 4; k++)
                        {
                            std::cout<<"[" << (*(recognizedContours[i*4+k]))[j][0] <<", "<< (*(recognizedContours[i*4+k]))[j][1] << "] ";
                        }
                        std::cout << std::endl;
                    }
                    std::cout << std::endl << std::endl;
                }
                std::cout << std::endl;
#endif

                // From extracted squared binary pattern, match the one corresponding to the squared binary marker
                if (patternMatcher->match(markerPatternDescriptor, recognizedPatternsDescriptors, patternMatches) == features::DescriptorMatcher::DESCRIPTORS_MATCHER_OK)
                {
#ifndef NDEBUG
                    std::cout << "Matches :" << std::endl;
                    for (int num_match = 0; num_match < patternMatches.size(); num_match++)
                        std::cout << "Match [" << patternMatches[num_match].getIndexInDescriptorA() << "," << patternMatches[num_match].getIndexInDescriptorB() << "], dist = " << patternMatches[num_match].getMatchingScore() << std::endl;
                    std::cout << std::endl << std::endl;
#endif

                    // Reindex the pattern to create two vector of points, the first one corresponding to marker corner, the second one corresponding to the poitsn of the contour
                    patternReIndexer->reindex(recognizedContours, patternMatches, pattern2DPoints, img2DPoints);
#ifndef NDEBUG
                    LOG_DEBUG("2D Matched points :");
                    for (int i = 0; i < img2DPoints.size(); i++)
                    LOG_DEBUG("{}",img2DPoints[i]);
                    for (int i = 0; i < pattern2DPoints.size(); i++)
                    LOG_DEBUG("{}",pattern2DPoints[i]);
                    overlay2DCircles->drawCircles(img2DPoints, inputImage);
#endif
                    // Compute the 3D position of each corner of the marker
                    img2worldMapper->map(pattern2DPoints, pattern3DPoints);
#ifndef NDEBUG
                    std::cout << "3D Points position:" << std::endl;
                    for (int i = 0; i < pattern3DPoints.size(); i++)
                    LOG_DEBUG("{}", pattern3DPoints[i]);
#endif
                    // Compute the pose of the camera using a Perspective n Points algorithm using only the 4 corners of the marker
                    if (PnP->estimate(img2DPoints, pattern3DPoints, pose) == FrameworkReturnCode::_SUCCESS)
                    {
                        LOG_DEBUG("Camera pose : \n {}", pose.matrix());
                        // Display a 3D box over the marker
                        overlay3D->draw(pose,inputImage);
                        cv::Mat display_mat4;
                        SolAR::MODULES::OPENCV::SolAROpenCVHelper::mapToOpenCV(inputImage,display_mat4);
                        cv::cvtColor( display_mat4,display_mat, CV_BGR2BGRA );
                    }
                }
            }
        }

        ANativeWindow_unlockAndPost(m_native_window);
        ANativeWindow_release(m_native_window);
    }
    FlipCamera();
}

void CV_Main::KPDetect(cv::Mat &frame) {

    std::vector<cv::KeyPoint> kpts;
    cv::Mat desc;
    cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();
    akaze->detectAndCompute(frame, cv::noArray(), kpts, desc);
    //cv::Mat inFrame;
    //inFrame=frame.clone();
    //cv::drawKeypoints(inFrame, kpts, frame,CV_RED,cv::DrawMatchesFlags::DEFAULT);

    for (int i=0; i<kpts.size();i++)
    {
        //cv::Point point(kpts.at(i).pt);
        cv::circle(frame, kpts.at(i).pt, 1, CV_GREEN, 4, 8, 0 );
    }
    //cv::Point center(100,100);
    //cv::circle(frame, center, 10, CV_RED, 4, 8, 0 );


}

void CV_Main::FaceDetect(cv::Mat &frame) {

  std::vector<cv::Rect> faces;
  std::vector<cv::Rect> cars;
  cv::Mat frame_gray;

  cv::cvtColor(frame, frame_gray, CV_RGBA2GRAY);

 // equalizeHist( frame_gray, frame_gray );

 //-- Detect faces

  face_cascade.detectMultiScale(frame_gray, faces, 1.18, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(70, 70));

  for( size_t i = 0; i < faces.size(); i++ ) {
    cv::Point center(faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5);

    ellipse(frame, center, cv::Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360,
            CV_PURPLE, 4, 8, 0);

    cv::Mat faceROI = frame_gray( faces[i] );
    std::vector<cv::Rect> eyes;

    //-- In each face, detect eyes
    eyes_cascade.detectMultiScale( faceROI, eyes, 1.2, 2, 0 |CV_HAAR_SCALE_IMAGE, cv::Size(45, 45) );

    for( size_t j = 0; j < eyes.size(); j++ ) {
      cv::Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
      int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
      circle( frame, center, radius, CV_RED, 4, 8, 0 );
    }
  }
/*
  cars_cascade.detectMultiScale(frame_gray, cars, 1.18, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(70, 70));
  for( size_t i = 0; i < cars.size(); i++ ) {
        cv::Point center(cars[i].x + cars[i].width * 0.5, cars[i].y + cars[i].height * 0.5);

        ellipse(frame, center, cv::Size(cars[i].width * 0.5, cars[i].height * 0.5), 0, 0, 360,
                CV_PURPLE, 4, 8, 0);
  }
*/
  end_t = clock();
  total_t += (double)(end_t - start_t) / CLOCKS_PER_SEC;
  LOGI("Current Time: %f", total_t);
  if (total_t >= 20) {
    // stop after 20 seconds
    LOGI("DONE WITH 20 SECONDS");
    scan_mode = false;
  }
  start_t = clock();

}


void CV_Main::NatDetect(cv::Mat &frame) {

}

void CV_Main::FidDetect(cv::Mat &frame) {

}


// When scan button is hit
void CV_Main::RunCV() {

    if (scan_mode)
    {
        scan_mode=false;
        return;
    }

    scan_mode = true;
    face_mode=false;
    nat_mode=false;
    fid_mode=false;
    total_t = 0;
    start_t = clock();
}

void CV_Main::RunCV_face() {

    if (face_mode)
    {
        face_mode=false;
        return;
    }

    face_mode = true;
    scan_mode=false;
    nat_mode=false;
    fid_mode=false;

    total_t = 0;
    start_t = clock();
}

void CV_Main::RunCV_nat() {

    if (nat_mode)
    {
        nat_mode=false;
        return;
    }

    nat_mode = true;
    scan_mode=false;
    face_mode=false;
    fid_mode=false;
    total_t = 0;
    start_t = clock();
}

void CV_Main::RunCV_fid() {

    if (fid_mode)
    {
        fid_mode=false;
        return;
    }

    fid_mode = true;
    scan_mode=false;
    nat_mode=false;
    face_mode=false;
    total_t = 0;
    start_t = clock();
}


void CV_Main::HaltCamera() {
  if (m_native_camera == nullptr) {
    LOGE("Can't flip camera without camera instance");
    return; // need to setup camera
  } else if (m_native_camera->GetCameraCount() < 2) {
    LOGE("Only one camera is available"); // TODO - remove button if this is true
    return; // need a second camera to flip with
  }

  m_camera_thread_stopped = true;
}

void CV_Main::FlipCamera() {
  m_camera_thread_stopped = false;

  // reset info
  if (m_image_reader != nullptr) {
    delete (m_image_reader);
    m_image_reader = nullptr;
  }
  delete m_native_camera;

  if (m_selected_camera_type == FRONT_CAMERA) {
    m_selected_camera_type = BACK_CAMERA;
  } else {
    m_selected_camera_type = FRONT_CAMERA;
  }

  SetUpCamera();

  std::thread loopThread(&CV_Main::CameraLoop, this);
  loopThread.detach();
}