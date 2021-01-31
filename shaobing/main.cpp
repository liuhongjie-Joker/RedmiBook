#include "thread"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "ImageConsProd.h"
#include "armordetection.h"
#include "mode_define.h"
using namespace cv;
using namespace std;

armor_detector ad(VIDEO_WIDTH, VIDEO_HEIGHT);

int main()
{
#ifdef CAR_REAL_SCENE
    // start threads
    ImageConsProd image_cons_prod;
//    image_cons_prod.ImageProducer();
//     利用多线程加快速度
    set_detector(&ad);
    std::thread t1(&ImageConsProd::ImageProducer, image_cons_prod); // pass by reference
    std::thread t2(&ImageConsProd::ImageConsumer, std::ref(image_cons_prod));

    t1.join();
    t2.join();
#endif
#ifdef RUN_VIDEO
    VisionData vdata;
    // 读取相机参数
    FileStorage fs(cameraParamsfile, FileStorage::READ);
    Mat cam, dis;
    fs["cameraMatrix"]>>cam;
    cout<<"相机内参: "<<endl;
    cout<<cam<<endl;
    fs["distCoeffs"]>>dis;
    cout<<"畸变系数: "<<endl;
    cout<<dis<<endl;
    fs.release();

    VideoCapture cap("/media/nuc/DISK_IMG/lhj/data/videoTest.avi");
    while(1)
    {
        cap>>ad.origin_img;
        if(ad.origin_img.empty())
        {
            cout<<"end of video!"<<endl;
            break;
        }
        ad.pre_frame = ad.now_frame;
        ad.origin_img.copyTo(ad.img);
        ad.set_ROI();
    //            GaussianBlur(ad.img, ad.img, Size(5,5),0);
        Mat threshold_image;
        threshold_image = ad.get_LightBar();
        vector<Point2f> mask_point;
        mask_point = ad.get_maskpoint(threshold_image);
        recognize_result res;
        ad.recognize_armoured_plate(mask_point, res);
        ad.now_frame.type = res.type;
        if(res.type!=NOT_Armoured_Plate)
        {
            ad.now_frame.armoured_plate = res.best_attack_object;
        }
        ad.pnp(cam,dis,res);
        ad.judge_is_spinning();
        ad.deal_with_spark();
        vdata.dis.f = ad.distance;
        vdata.pitch_angle.f = ad.pitch;// + 1.5;
        vdata.yaw_angle.f = ad.yaw;// - 3.5;
        if(ad.pitch_vec.size()>5)
        {
            vdata.isFindTarget = res.type>0?1:0;
        }
        vdata.buff_change_four = 1;
        vdata.anti_top = 1;
        vdata.anti_top_change_armor = 1;
        vdata.nearFace = 1;
        vdata.isfindDafu= 1;
#ifdef DEBUG_COUT
        temp(res);
        cout<<res.best_attack_object.size<<endl;
        cout<<"#########################################################"<<endl;
#endif
#ifdef COUT_TRANSFORM_DATA
    cout<<detector->key<<endl;
    cout<<vdata.dis.f<<" "<<vdata.pitch_angle.f<<" "<<vdata.yaw_angle.f<<endl;
#endif
#ifdef SHOW_IMAGE
        ad.mask_armoured_plate(res);
        ad.show_img(threshold_image, vdata);
#endif
    }
#endif
}
