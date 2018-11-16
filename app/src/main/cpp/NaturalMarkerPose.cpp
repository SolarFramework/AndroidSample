//
// Created by atadrist on 31/10/18.
//

#include "NaturalMarkerPose.h"

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


NaturalMarkerPose::NaturalMarkerPose() {}

int NaturalMarkerPose::setup()
{

/* instantiate component manager*/
    /* this is needed in dynamic mode */
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    if(xpcfComponentManager->load("/sdcard/solar/conf_NaturalImageMarker.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file conf_NaturalImageMarker.xml")
        return -1;
    }
    // declare and create components
    LOG_INFO("Start creating components");

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
    if (!camera || !imageViewerKeypoints || !imageViewerResult || !marker || !kpDetector || !descriptorExtractor || !matcher || !basicMatchesFilter || !geomMatchesFilter || !homographyEstimation ||
        !homographyValidation ||!keypointsReindexer || !poseEstimation || !overlay2DComponent || !overlay3DComponent || !img_mapper || !transform2D )
    {
        LOG_ERROR("One or more component creations have failed");
        return -1;
    }
    LOG_INFO("All components have been created");

    // the following code is common to the 3 samples (simple, compile-time, run-time)
    //
    //
    // Declare data structures used to exchange information between components
    SRef<Image> refImage, camImage, kpImageCam;
    SRef<DescriptorBuffer> refDescriptors, camDescriptors;
    std::vector<DescriptorMatch> matches;

    Transform2Df Hm;
    std::vector< SRef<Keypoint> > refKeypoints, camKeypoints;  // where to store detected keypoints in ref image and camera image

    // load marker
    LOG_INFO("LOAD MARKER IMAGE ");
    marker->loadMarker();
    marker->getImage(refImage);

    // NOT WORKING ! Set the size of the box to the size of the natural image marker
    overlay3DComponent->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(marker->getWidth(),0);
    overlay3DComponent->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(marker->getHeight(),1);
    overlay3DComponent->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(marker->getHeight()/2.0f,2);

    // detect keypoints in reference image
    LOG_INFO("DETECT MARKER KEYPOINTS ");
    kpDetector->detect(refImage, refKeypoints);

    // extract descriptors in reference image
    LOG_INFO("EXTRACT MARKER DESCRIPTORS ");
    descriptorExtractor->extract(refImage, refKeypoints, refDescriptors);
    LOG_INFO("EXTRACT MARKER DESCRIPTORS COMPUTED");

#ifndef NDEBUG
    // display keypoints in reference image
    // copy reference image
    SRef<Image> kpImage = refImage->copy();
    // draw circles on keypoints

    overlay2DComponent->drawCircles(refKeypoints, kpImage);
    // displays the image with circles in an imageviewer
    imageViewerKeypoints->display(kpImage);
#endif

    if (camera->start() != FrameworkReturnCode::_SUCCESS) // videoFile
    {
        LOG_ERROR("Camera cannot start");
        return -1;
    }

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

    return 0;
}



NaturalMarkerPose::~NaturalMarkerPose() {}