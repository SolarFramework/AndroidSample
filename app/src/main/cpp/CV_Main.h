#ifndef OPENCV_NDK_CV_MAIN_H
#define OPENCV_NDK_CV_MAIN_H

#include "NaturalMarkerPose.h"

// Android
#include <android/asset_manager.h>
#include <android/native_window.h>
#include <jni.h>
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/features2d.hpp>
// OpenCV-NDK App
#include "ImageReader.h"
#include "NativeCamera.h"
#include "Util.h"
// C Libs
#include <unistd.h>
#include <time.h>
// STD Libs
#include <cstdlib>
#include <string>
#include <vector>
#include <thread>

class CV_Main {
 public:
  CV_Main();
  ~CV_Main();
  CV_Main(const CV_Main& other) = delete;
  CV_Main& operator=(const CV_Main& other) = delete;

  // Lets us know when app has started passing in VM info
  void OnCreate(JNIEnv* env, jobject caller_activity);

  // TODO
  // Disconnect from service
  void OnPause();

  // TODO
  // Cleanup
  void OnDestroy();

  // Cache the Java VM used from the Java layer.
  void SetJavaVM(JavaVM* pjava_vm) { java_vm = pjava_vm; }

  // sets Surface buffer reference pointer
  void SetNativeWindow(ANativeWindow* native_window);

  // sets Surface buffer reference pointer
  void SetAssetManager(AAssetManager* asset_manager) {
    m_aasset_manager = asset_manager;
  };

  void SetUpCamera();

  void CameraLoop();
  void SolARCameraLoop_full();
  void SolARCameraLoop();


  void FaceDetect(cv::Mat &frame);
  void KPDetect(cv::Mat &frame);
  void NatDetect(cv::Mat &frame);
  void FidDetect(cv::Mat &frame);

  void RunCV();
  void RunCV_face();
  void RunCV_nat();
  void RunCV_fid();

  void HaltCamera();
  void FlipCamera();

 private:
  // Cached Java VM, caller activity object
  JavaVM* java_vm;
  jobject calling_activity_obj;
  jmethodID on_callback;

  // holds native window to write buffer too
  ANativeWindow* m_native_window;

  // buffer to hold native window when writing to it
  ANativeWindow_Buffer m_native_buffer;

  // Camera variables
  NativeCamera* m_native_camera;

  camera_type m_selected_camera_type = BACK_CAMERA; // Default

  // Image Reader
  ImageFormat m_view{0, 0, 0};
  ImageReader* m_image_reader;
  AImage* m_image;

  volatile bool m_camera_ready;

  // used to hold reference to assets in assets folder
  AAssetManager* m_aasset_manager;

  // for timing OpenCV bottlenecks
  clock_t start_t, end_t;
  double  total_t;

  // Used to detect up and down motion
    bool scan_mode=false;
    bool face_mode=false;
    bool nat_mode=false;
    bool fid_mode=false;

  // OpenCV values
  cv::Mat display_mat;
  // Currently no way of getting file string for load() call, need to manually
  // store the assets in the sdcard and grab them from there
  cv::String face_cascade_name = "/data/data/com.example.atadrist.opencvnewsample/opencv/haarcascade_frontalface_alt.xml";
  cv::String eyes_cascade_name = "/data/data/com.example.atadrist.opencvnewsample/opencv/haarcascade_eye_tree_eyeglasses.xml";
  cv::String cars_cascade_name = "/data/data/com.example.atadrist.opencvnewsample/opencv/cars.xml";
  cv::CascadeClassifier face_cascade;
  cv::CascadeClassifier eyes_cascade;
  cv::CascadeClassifier cars_cascade;


  cv::Scalar CV_PURPLE = cv::Scalar ( 255, 0, 255 );
  cv::Scalar CV_RED = cv::Scalar ( 255, 0, 0 );
  cv::Scalar CV_GREEN = cv::Scalar ( 0, 255, 0 );
  cv::Scalar CV_BLUE = cv::Scalar ( 0, 0, 255 );

  bool m_camera_thread_stopped = false;
};

#endif  // OPENCV_NDK_CV_MAIN_H
