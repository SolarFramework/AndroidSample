package com.example.atadrist.opencvnewsample;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.TextView;
import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager;
import android.content.res.AssetManager;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.support.v4.app.ActivityCompat;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.widget.Button;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import android.content.pm.ApplicationInfo;
import android.content.Intent;



import android.widget.TextView;

public class MainActivity extends AppCompatActivity {

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
    }

    static final String TAG = "azdine-sample";


    private static final int PERMISSION_REQUEST_CODE_CAMERA = 1;

    Button mScanButton;
    Button mFaceButton;
    Button mNatButton;
    Button mFidButton;
    Button mFlipCameraButton;
    SurfaceView mSurfaceView;
    SurfaceHolder mSurfaceHolder;


    /*
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Example of a call to a native method
        TextView tv = (TextView) findViewById(R.id.sample_text);
        tv.setText(stringFromJNI());
    }
    */
    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        /////////// NEW
        // Asking for permissions
        String[] accessPermissions = new String[] {
                Manifest.permission.CAMERA,
                Manifest.permission.INTERNET,
                Manifest.permission.WRITE_EXTERNAL_STORAGE,
                Manifest.permission.READ_EXTERNAL_STORAGE,
        };
        boolean needRequire = false;
        for(String access : accessPermissions) {
            int curPermission = ActivityCompat.checkSelfPermission(this, access);
            if (curPermission != PackageManager.PERMISSION_GRANTED) {
                needRequire = true;
                break;
            }
        }
        if (needRequire) {
            ActivityCompat.requestPermissions(
                    this,
                    accessPermissions,
                    PERMISSION_REQUEST_CODE_CAMERA);
            // copy config files only 1st launch, when permissions are granted
            Log.i("tag","copy assets folder to internal storage");
            copyFileOrDir(getDataDir().getAbsolutePath(), "");
            // copy libs too
            String libpath = getApplicationInfo().nativeLibraryDir;
            Log.i("tag","##### library path is: " + libpath);
            String datapath= getApplicationInfo().dataDir;
            Log.i("tag","##### data path is: " + datapath);
            try {
                Process symlink = Runtime.getRuntime().exec("ln -s " + libpath + " " + datapath + "/lib");
            }
            catch (IOException e)
            {
                Log.i("error","##### ln -s failed");
            }
            return;
        }



        // send class activity and assest fd to native code
        onCreateJNI(this, getAssets());
        // set up the Surface to display images too
        mSurfaceView = (SurfaceView) findViewById(R.id.surfaceView);
        mSurfaceHolder = mSurfaceView.getHolder();

        mScanButton = (Button)findViewById(R.id.scan_button);
        mScanButton.setOnClickListener(scanListener);

        mFaceButton = (Button)findViewById(R.id.face_button);
        mFaceButton.setOnClickListener(faceListener);

        mNatButton = (Button)findViewById(R.id.nat_button);
        mNatButton.setOnClickListener(natListener);

        mFidButton = (Button)findViewById(R.id.fid_button);
        mFidButton.setOnClickListener(fidListener);

        mFlipCameraButton = (Button)findViewById(R.id.flip_button);
        mFlipCameraButton.setOnClickListener(flipCameraListener);

        mSurfaceHolder.addCallback(new SurfaceHolder.Callback() {

            public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
                Log.v(TAG, "surfaceChanged format="+format+", width="+width+", height="+ height);
            }

            public void surfaceCreated(SurfaceHolder holder) {
                Log.v(TAG, "surfaceCreated");
                setSurface(holder.getSurface());
            }

            public void surfaceDestroyed(SurfaceHolder holder) {
                Log.v(TAG, "surfaceDestroyed");
            }

        });

        CameraManager manager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
        try
        {
            for (String cameraId : manager.getCameraIdList()) {
                CameraCharacteristics characteristics
                        = manager.getCameraCharacteristics(cameraId);

                Log.d("Img", "INFO_SUPPORTED_HARDWARE_LEVEL " + characteristics.get(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL));
                Log.d("Img", "INFO_REQUIRED_HARDWARE_LEVEL FULL" + CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_FULL);
                Log.d("Img", "INFO_REQUIRED_HARDWARE_LEVEL 3" + CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_3);
                Log.d("Img", "INFO_REQUIRED_HARDWARE_LEVEL LIMITED" + CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED);
                Log.d("Img", "INFO_REQUIRED_HARDWARE_LEVEL LEGACY" + CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY);



            }
        }
        catch (CameraAccessException e){
            e.printStackTrace();
        }
    }


    private View.OnClickListener scanListener = new View.OnClickListener() {
        @Override
        public void onClick(View view) {
            scan();
        }
    };
    private View.OnClickListener faceListener = new View.OnClickListener() {
        @Override
        public void onClick(View view) {
            face();
        }
    };
    private View.OnClickListener natListener = new View.OnClickListener() {
        @Override
        public void onClick(View view) {
            nat();
        }
    };
    private View.OnClickListener fidListener = new View.OnClickListener() {
        @Override
        public void onClick(View view) {
            fid();
        }
    };

    private View.OnClickListener flipCameraListener = new View.OnClickListener() {
        @Override
        public void onClick(View view) {
            flipCamera();
        }
    };

    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     */
    public native String stringFromJNI();


    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     */
    public native void onCreateJNI(Activity callerActivity, AssetManager assetManager);

    public native void scan();
    public native void face();
    public native void nat();
    public native void fid();



    public native void flipCamera();

    // Sends surface buffer to NDK
    public native void setSurface(Surface surface);

    ///
    private void copyFileOrDir(String TARGET_BASE_PATH, String path) {
        AssetManager assetManager = this.getAssets();
        String assets[] = null;
        try {
            Log.i("tag", "copyFileOrDir() "+path);
            assets = assetManager.list(path);
            if (assets.length == 0) {
                copyFile(TARGET_BASE_PATH, path);
            } else {
                String fullPath =  TARGET_BASE_PATH + "/" + path;
                Log.i("tag", "path="+fullPath);
                File dir = new File(fullPath);
                if (!dir.exists() && !path.startsWith("images") && !path.startsWith("sounds") && !path.startsWith("webkit"))
                    if (!dir.mkdirs())
                        Log.i("tag", "could not create dir "+fullPath);
                for (int i = 0; i < assets.length; ++i) {
                    String p;
                    if (path.equals(""))
                        p = "";
                    else
                        p = path + "/";

                    if (!path.startsWith("images") && !path.startsWith("sounds") && !path.startsWith("webkit"))
                        copyFileOrDir(TARGET_BASE_PATH, p + assets[i]);
                }
            }
        } catch (IOException ex) {
            Log.e("tag", "I/O Exception", ex);
        }
    }

    private void copyFile(String TARGET_BASE_PATH, String filename) {
        AssetManager assetManager = this.getAssets();

        InputStream in = null;
        OutputStream out = null;
        String newFileName = null;
        try {
            Log.i("tag", "copyFile() "+filename);
            in = assetManager.open(filename);
            newFileName = TARGET_BASE_PATH + "/" + filename;
            out = new FileOutputStream(newFileName);

            byte[] buffer = new byte[1024];
            int read;
            while ((read = in.read(buffer)) != -1) {
                out.write(buffer, 0, read);
            }
            in.close();
            in = null;
            out.flush();
            out.close();
            out = null;
        } catch (Exception e) {
            Log.e("tag", "Exception in copyFile() of "+newFileName);
            Log.e("tag", "Exception in copyFile() "+e.toString());
        }

    }
}
