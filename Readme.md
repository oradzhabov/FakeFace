# FakeFace

Research project for testing the techniques of processing the human face images via the real-time video streaming.

It uses DirectShow filters for grabing video frames from real camera device, processing them and passing to the virtual camera.

During processing of frames it uses dlib or parts of OpenFace to putting any human face from image to corresponded face on the frame.

![Alt text](https://github.com/oradzhabov/FakeFace/blob/master/img/FakeFace_demo.gif?raw=true "Title")

# Minimum Requirements

* Windows OS (x86 & x64)
* Microsoft Visual Studio 2010


# Dependencies

## OpenCV
* web: http://opencv.org/
* notes:
    1. I've used OpenCV 3.1

## dlib
* web: http://dlib.net/
* github: https://github.com/davisking/dlib
* notes:
    1. dlib should not be just compiled but installed. Hence in my case, when I point dlib to install in local folder, I have next path for cmake entry __dlib_DIR__ D:/DLIB/dlib-18.18/build/msvc2010.x32/install/lib/cmake/dlib
    2. I've used dlib 18.18

## boost
* web: http://www.boost.org/
* notes:
    1. I've used boost 1.54

## Microsoft Windows SDK

# Build
Use only release target

# Prepare

## Register new virtual camera
* Run __FAR__ or __CMD__ with Administrator permissions
* Point to folder where file _VirtualCamDevice.dll_ has been built
* During building there are several files has been copied to the same folder
* Run file _install.bat_
* If no errors occured during registration system will inform about it twice (actually 2 files will be registered)

## Unregister virtual camera
* If you do not want to continue using the virtual camera, run file _uninstall.bat_ from the folder where file _VirtualCamDevice.dll_ has been built
* Ensure that no applications (GraphStudio, Skype, Hangout or else) do not use virtual camera. Otherwise it will not uninstall it

## Run
* Ensure that no application uses your real web-camera device. Otherwise it could not be stated as imput source for FakeFace
* Run file _VirtualCameraManager.exe_
* After several seconds FakeFace manager window will apear
* From this moment you can use FakeFace

# Using Cases

## GraphStudio
* For testing purposes I use __Graphstudio__ (http://blog.monogram.sk/janos/2009/06/14/monogram-graphstudio-0320/)
* Notice that if you build FakeFace for x64 platform, use __GraphStudio64__ instead of __GraphStudio__
* Download and run it. Open project "./data/MONOGRAM GraphStudio/FakeFace Virtual Cam.grf"
* This project connect virtual camera (registered before) and renderer
* Start the graph
* If you did not start VirtualCamManager, renderer will show noisy pixels
* If VirtualCamManager has been started before, you will see your web-camera stream where your face will be switched to face of Vasyl Lomachenko - default fake face
* Pick to the __FakeFace Manager__ window and press "S"-button on the keyboard
* OpenFile dialog will appear. Choose some file with human face photo
* After moment your face on the GraphStudio renderer will changed to that which imaged on the selected file

## Skype
* Make sure that the destination platform of the installed version of Skype corresponds to the build destination platform of FakeFace
* If you wish to use FakeFace with Skype, you need to register virtual camera when Skype is turned off. Remember that virtual camera should be registered when no one application use your real camera device
* After virtual camera has been registered, VirtualCamManager has been started and Skype has been stared pick Skype menu: Tools >> Options >> Video settings
* Your default camera will not be available
* On the panel "Video settings" select webcam: "FakeFace Virtual Cam"
* Enjoy
    
# Tips
* Avoid back lighting. Set light direction frontal to face
* Use clothes that cover your neck. Color of your face shold not interfere with color of neck as background color