#include <iostream>
#include <map>
#include <limits>
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/viz.hpp"

using namespace std;
using namespace cv;

class myClass{
public:
    /* I used a Hashmap / map because it is easier to call special items and needs less Time.
     * Lists: Iterrate as long as I found the item --> Map: call the item.
     * */
    static void GetIntrinsics(map<string, Mat> &myMap, char* path){
        FileStorage myfs(path, FileStorage::READ);
        
        if(!myfs.isOpened()){
            cout << "Error by Opening FileStorage" << endl;
            return;
        }
        //Save intrinsic Mat Objects
        myfs["M1"] >> myMap["M1"];
        myfs["D1"] >> myMap["D1"];
        myfs["M2"] >> myMap["M2"];
        myfs["D2"] >> myMap["D2"];
    };
    
    static void GetExtrinsics(map<string, Mat> &myMap, char* path){
        FileStorage myfs(path, FileStorage::READ);
        
        if(!myfs.isOpened()){
            cout << "Error by Opening FileStorage" << endl;
            return;
        }
        ///Save extrinsic Mat Objects
        myfs["R"]   >> myMap["R"];
        myfs["T"]   >> myMap["T"];
        myfs["R1"]  >> myMap["R1"];
        myfs["R2"]  >> myMap["R2"];
        myfs["P1"]  >> myMap["P1"];
        myfs["P2"]  >> myMap["P2"];
        myfs["Q"]   >> myMap["Q"];
    };
    
    static void printMap(map<string, Mat> &myMap){
        for (map<string, Mat>::const_iterator it = myMap.begin(); it != myMap.end(); it++)
            cout << it->first << ": " << endl << it->second << endl;
    }
};

int main(int argc, char** argv )
{     
    //-----------------------------------------------------------------------------------------------------------------------
    if( argc != 5){
       cerr << "Wrong Parameter. Usage: ./[Program name] [left image] [right image] [yml intrinsics] [yml extrinsics]" << endl;
       return -1;
    }
    
    char* ImageLeft = argv[1];
    char* ImageRight = argv[2];
    char* intrinsic = argv[3];
    char* extrinsic = argv[4];
    
    cout << ImageLeft << endl << ImageRight << endl << intrinsic << endl << extrinsic << endl;
    
    Mat imageL = imread( ImageLeft, IMREAD_UNCHANGED );
    Mat imageR = imread( ImageRight, IMREAD_UNCHANGED );
    
    if(!imageL.data | !imageR.data){
        cout << "Cannot read an image file " << endl;
        return -1;
    }
    //-----------------------------------------------------------------------------------------------------------------------
    Mat BWImage;
    
    /*
     * P1:  8*number_of_image_channels*SADWindowSize*SADWindowSize --> 8*1*3*3
     * P2:  32*number_of_image_channels*SADWindowSize*SADWindowSize --> 64*1*3*3
     * */
    Ptr<StereoSGBM> test = StereoSGBM::create(0 ,16*2, 5, 8*20*3*3, 64*1*3*3, 0, 0, 12.5, 0, 0, StereoSGBM::MODE_SGBM);

    map<string, Mat> intrMap;
    map<string, Mat> extrMap;
    
    myClass::GetIntrinsics(intrMap, intrinsic);
    myClass::GetExtrinsics(extrMap, extrinsic);
    myClass::printMap(intrMap);
    myClass::printMap(extrMap);
    
    Mat left, right;
    /*LEFT IMAGE*/
    stereoRectify(intrMap["M1"], intrMap["D1"], intrMap["M2"], intrMap["D2"], imageL.size(), extrMap["R"], extrMap["T"], 
                    extrMap["R1"], extrMap["R2"], extrMap["P1"], extrMap["P2"], extrMap["Q"], CALIB_ZERO_DISPARITY, -1, imageL.size());
    Mat Lmap1, Lmap2;
    initUndistortRectifyMap(intrMap["M1"], intrMap["D1"], extrMap["R1"], extrMap["P1"], imageL.size(), CV_32FC1, Lmap1, Lmap2);
    remap(imageL, left, Lmap1, Lmap2, INTER_LINEAR);
    
    /*RIGHT IMAGE*/
    stereoRectify(intrMap["M1"], intrMap["D1"], intrMap["M2"], intrMap["D2"], imageR.size(), extrMap["R"], extrMap["T"], 
                    extrMap["R1"], extrMap["R2"], extrMap["P1"], extrMap["P2"], extrMap["Q"], CALIB_ZERO_DISPARITY, -1, imageR.size());
    Mat Rmap1, Rmap2;
    initUndistortRectifyMap(intrMap["M1"], intrMap["D1"], extrMap["R1"], extrMap["P1"], imageL.size(), CV_32FC1, Rmap1, Rmap2);
    remap(imageR, right, Rmap1, Rmap2, INTER_LINEAR);
        
    test.get()->compute(left, right, BWImage);
    
    Mat img3D;
    reprojectImageTo3D(BWImage, img3D, extrMap["Q"], false, -1);
    
    viz::Viz3d visual3D("3D Image");
    
    /* Mathias Lackner brought me to the idea to use "Point3f" and showed me the generell usage.
     * */
    for(MatIterator_<Point3f> it = img3D.begin<Point3f>(); it != img3D.end<Point3f>(); it++ ){
        Point3f p = (*it);
        if((p.z > 7))
            *it = Point3f(numeric_limits<float>::quiet_NaN(),numeric_limits<float>::quiet_NaN(),numeric_limits<float>::quiet_NaN());
        else if(p.z < 0)
            *it = Point3f(0,0,0);
    }

    viz::WCloud cloud(img3D, left);
    
    visual3D.showWidget("left3DImg", cloud);
    
    Affine3<double> a = visual3D.getViewerPose();
    a = a.rotate(Vec3d(0,0,CV_PI));
    a = a.rotate(Vec3d(0,CV_PI,0));
    visual3D.setViewerPose(a);
    
    visual3D.spin();
    
    return 0;
}