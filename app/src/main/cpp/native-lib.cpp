#include <jni.h>
#include <string>
#include <opencv2/core.hpp>
#include <core/SolARFramework.h>
#include "datastructure/CloudPoint.h"
#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"
#include <boost/test/unit_test.hpp>

#include "SolARModuleOpencv_traits.h"
#include "api/input/devices/ICamera.h"
#include "api/display/IImageViewer.h"
#include "api/input/files/IMarker2DNaturalImage.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/pose/I2DTransformFinder.h"
#include "api/solver/pose/IHomographyValidation.h"
#include "api/features/IKeypointsReIndexer.h"
#include "api/solver/pose/I3DTransformFinderFrom2D2D.h"
#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"
#include "api/solver/pose/I2D3DCorrespondencesFinder.h"
#include "api/display/I2DOverlay.h"
#include "api/display/IMatchesOverlay.h"
#include "api/display/I3DOverlay.h"
#include "api/geom/IImage2WorldMapper.h"
#include "api/geom/I2DTransform.h"
#include "api/features/IContoursExtractor.h"
#include "api/features/IContoursFilter.h"
#include "api/features/IDescriptorsExtractorSBPattern.h"
#include "api/solver/pose/I2Dto3DTransformDecomposer.h"
#include "api/solver/pose/I3DTransformFinderFrom2D2D.h"
#include "api/image/IImageConvertor.h"
#include "api/image/IImageFilter.h"
#include "api/image/IImageLoader.h"
#include "api/input/files/IMarker2DSquaredBinary.h"
#include "api/image/IPerspectiveController.h"
#include "api/solver/map/ITriangulator.h"
#include "api/solver/map/IMapFilter.h"
#include "api/solver/map/IMapper.h"
#include "api/input/devices/ICameraCalibration.h"

#include <android/asset_manager_jni.h>
#include <android/native_window_jni.h>
#include <thread>
#include "CV_Main.h"

namespace xpcf  = org::bcom::xpcf;

using namespace SolAR;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;

static CV_Main app;

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT jstring JNICALL
Java_com_example_atadrist_opencvnewsample_MainActivity_stringFromJNI(
        JNIEnv *env,
        jobject /* this */) {
        //std::string hello = "Hello from C++";
    std::string hello = cv::getBuildInformation();

    // some solar code here
    SolAR::datastructure::Point2Df outputPoint2D;
    // load library
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    return env->NewStringUTF(hello.c_str());
}


jint JNI_OnLoad(JavaVM *vm, void *) {
    // We need to store a reference to the Java VM so that we can call back
    app.SetJavaVM(vm);
    return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL
Java_com_example_atadrist_opencvnewsample_MainActivity_onCreateJNI(
        JNIEnv *env, jobject clazz, jobject activity, jobject j_asset_manager) {
    app.OnCreate(env, activity);
    app.SetAssetManager(AAssetManager_fromJava(env, j_asset_manager));
}

JNIEXPORT void JNICALL Java_com_example_atadrist_opencvnewsample_MainActivity_scan(
        JNIEnv *env, jobject clazz) {
    app.RunCV();
}

JNIEXPORT void JNICALL Java_com_example_atadrist_opencvnewsample_MainActivity_face(
        JNIEnv *env, jobject clazz) {
    app.RunCV_face();
}

JNIEXPORT void JNICALL Java_com_example_atadrist_opencvnewsample_MainActivity_nat(
        JNIEnv *env, jobject clazz) {
    app.RunCV_nat();
}

JNIEXPORT void JNICALL Java_com_example_atadrist_opencvnewsample_MainActivity_fid(
        JNIEnv *env, jobject clazz) {
    app.RunCV_fid();
}

JNIEXPORT void JNICALL Java_com_example_atadrist_opencvnewsample_MainActivity_flipCamera(
        JNIEnv *env, jobject clazz) {
    app.HaltCamera();
}

// Alot of stuff depends on the m_frame_buffer being loaded
// this is done in SetNativeWindow
JNIEXPORT void JNICALL
Java_com_example_atadrist_opencvnewsample_MainActivity_setSurface(JNIEnv *env,
                                                               jclass clazz,
                                                               jobject surface) {
    // obtain a native window from a Java surface
    app.SetNativeWindow(ANativeWindow_fromSurface(env, surface));

    // Set camera parameters up
    app.SetUpCamera();
    LOGI("--(!)BEFORE STARTING LOOP THREAD\n");
    std::thread loopThread(&CV_Main::CameraLoop, &app);
    loopThread.detach();
    LOGI("--(!)Camera Set up OK\n");
}

#ifdef __cplusplus
}
#endif
