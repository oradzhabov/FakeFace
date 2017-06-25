![Alt text](https://github.com/oradzhabov/FakeFace/blob/master/img/FakeFace_demo.gif?raw=true "Title")

# Minimum Requirements

* Windows OS
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
    1. dlib should be not just compiled but installed. Hence in my case, when I point dlib to install in local folder, I have next path for cmake entry __dlib_DIR__ D:/DLIB/dlib-18.18/build/msvc2010.x32/install/lib/cmake/dlib

## boost
* web: http://www.boost.org/
* notes:
    1. I've used boost 1.54

## DirectX SDK with samples
* DirectShow

# Build
Use only release build

# Prepare

## Register new virtual camera
* Run FAR or CMD with Administrator permissions
* Point to folder where file _VirtualCamDevice.dll_ has been built
* During building there are several files has been copied to the same folder
* Run file _install.bat_
* If no errors occured during registration system will inform about it twice (actually 2 files should be registered)

## Unregister virtual camera
* If you do not want to continue using the virtual camera, run file _uninstall.bat_ from the folder where file _VirtualCamDevice.dll_ has been built
* Ensure that no other applications (GraphStudio, Skype, Hangout or else) do not use it. Otherwise it will not uninstall it

## Run
* Ensure that no application uses your camera device. Otherwise it could not be stated as imput source for FakeFace
* Run file _VirtualCameraManager.exe_
* After several seconds manager window will apear
* From this moment you can use FakeFace

# Using

## GraphStudio
* For testing purposes I use __Graphstudio__ (http://blog.monogram.sk/janos/tools/monogram-graphstudio/)
* Install and run it. Open project "./data/MONOGRAM GraphStudio/FakeFace Virtual Cam.grf"
* This project connect virtual camera (registered before) and renderer
* Start the graph
* If you did not start VirtualCamManager, renderer will show noisy pixels
* If VirtualCamManager has been started before, you will see your web-camera stream where your face will be switched to face of Vasily Lomachenko - default fake face
* Pick to caption panel of the __FakeFace Manager__ and press "S"-button on the keyboard
* OpenFile dialog will be appear. Choose some file with photo some human
* After several seconds, your face on the GraphStudio renderer will changed to that which imaged on the new file

## Skype
* If you wish to use FakeFace with skype, you need to register virtual camera when Skype is turned off. Remember that virtual camera shoudl be registered when no one application use your real camera device
* After virtual camera has been registered, VirtualCamManager has been started and Skype has been stared pick Skype menu: Tools >> Options >> Video settings
* Your default camera will not be available
* On the panel "Video settings" select webcam: "FakeFace Virtual Cam"
* Enjoy
    
