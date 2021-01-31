#include<iostream>
#include<qapplication.h>
#include<qdir.h>
#include<opencv2/opencv.hpp>
#include"GxIAPI.h"
#include "DxImageProc.h"
#include<ImageConsProd.h>
#include<armordetection.h>
#include<rp_kalman.h>
#include<queue>
#include<serialport.h>
#include<CRC_Check.h>
#include<Energy.h>
#include "rp_kalman.h"
#include "mode_define.h"
using namespace cv;
using namespace std;


int mode = 1, buff_four, shoot_speed,
my_color,car_yaw_angel,car_pitch_angel,
yaw_angel_speed,pitch_angel_speed;

//相机参数的设定
GX_DEV_HANDLE hDevice = NULL;   //设备句柄



volatile unsigned int prdIdx;
volatile unsigned int csmIdx;

#define BUFFER_SIZE 1


void set_detector(armor_detector *d)
{
    detector=d;
}

struct ImageData {
    Mat img;
    unsigned int frame;
};
ImageData data[BUFFER_SIZE];

SerialPort port("/dev/ttyUSB0");
VisionData vdata;


//回调函数
static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame)
{
    detector->ts.start = GetTickCount();
    int64_t nWidth;
    int64_t nHeight;

    nWidth = pFrame->nWidth;
    nHeight = pFrame->nHeight;

    nWidth = 1280;
    nHeight = 1024;

    uchar* m_pBufferRaw = new uchar[nWidth * nHeight * 3]; ///< 原始图像数据（内存空间）
    uchar* pRGB24Buf = new uchar[nWidth * nHeight * 3];    ///< RGB图像数据（内存空间）
    
    if (pFrame->status == 0)                               // 正常帧，残帧返回-1
    {
        Mat src;
        src.create(VIDEO_HEIGHT, VIDEO_WIDTH, CV_8UC3);

        memcpy(m_pBufferRaw, pFrame->pImgBuf, pFrame->nImgSize);
        DxRaw8toRGB24(m_pBufferRaw, pRGB24Buf, (VxUint32)(pFrame->nWidth), (VxUint32)(pFrame->nHeight), RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(1), false);
        memcpy(src.data, pRGB24Buf, nHeight * nWidth * 3);
        detector->ts.end = GetTickCount();
        detector->ts.add_image_collect_time();
        cvtColor(src, src, CV_RGB2BGR);

        while (prdIdx - csmIdx >= BUFFER_SIZE)  // 不能注释，注释了会出现零图，然后导致掉帧发生
        {
            if(detector->key == 'c')
            {
                break;
            }
        };

        data[prdIdx % BUFFER_SIZE].img = src;
//        vw << src;
//        imshow("src",src);
        GXFlushQueue(hDevice);  //
        prdIdx++;
//        cout<<prdIdx<<endl;

    }
    delete[] m_pBufferRaw;
    delete[] pRGB24Buf;
    return;

}

int assert_success(GX_STATUS status)
{
    if (status != GX_STATUS_SUCCESS)
    {
        status = GXCloseDevice(hDevice);
        if (hDevice != NULL)
        {

        }
        status = GXCloseLib();
        return 0;
    }
}


void ImageConsProd::ImageProducer()
{
    // 主相机初始化
    GX_STATUS status = GX_STATUS_SUCCESS;
    GX_OPEN_PARAM stOpenParam;
    uint32_t nDeviceNum = 0;

    cout << "主相机初始化中......" << endl;

    status = GXCloseLib();
    status = GXInitLib();


    if (status != GX_STATUS_SUCCESS)
    {
        cout << "初始化失败!" << endl;
        cout << "错误码：" << status << endl;
        cout<<111<<endl;
        return ;
    }
    cout << "主相机初始化完成!" << endl;

    status = GXUpdateDeviceList(&nDeviceNum, 1000);
    if ((status != GX_STATUS_SUCCESS) || (nDeviceNum <= 0))
    {
        cout << "获取设备列表失败" << endl;
        return ;
        cout<<222<<endl;
    }
    cout << "设备数：" << nDeviceNum << endl;

    stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
    stOpenParam.openMode = GX_OPEN_INDEX;
    stOpenParam.pszContent = "1";
    status = GXOpenDevice(&stOpenParam, &hDevice);

    if (status == GX_STATUS_SUCCESS)
    {
        //设置采集模式为连续采集
        status = GXSetEnum(hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
        assert_success(status);

        double val = 0;
        status = GXGetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, &val);

        ////DUBUG_CHANGE_EXPOSURE
        cout << "默认曝光值：" << val << endl;
        val = EXPOSURE;
        cout << "val = " << val << endl;
        status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, val);
        if (status == GX_STATUS_SUCCESS)
        {
            cout << "设置后曝光值：" << val << endl;
            assert_success(status);
        }
        else
            cout << "SET_EXPOSURE_FAIL" << endl;

        status = GXSetInt(hDevice, GX_INT_HEIGHT, VIDEO_HEIGHT);
        status = GXSetInt(hDevice, GX_INT_WIDTH, VIDEO_WIDTH);
//        status = GXSetInt(hDevice, GX_INT_OFFSET_X, 100);
//        status = GXSetInt(hDevice, GX_INT_OFFSET_Y, 304);
        assert_success(status);

#ifdef GAIN_SET
        //GAIN_SET
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
        if (status == GX_STATUS_SUCCESS){
            cout << "set_gain_succeed " << endl;
            assert_success(status);
        }else{
            cout << "增益_SELECT_fail:  " << status  << endl;
        }
        status = GXSetFloat(hDevice, GX_FLOAT_GAIN, GAIN);
        if (status == GX_STATUS_SUCCESS){
            cout << "set_gain_succeed_ggg " << endl;
            assert_success(status);
        }else{
            cout <<"增益 fail:  "  << status <<endl;
        }

#endif

#ifdef WHITEBALANCE_SET

        status = GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
        if (status == GX_STATUS_SUCCESS){
            cout <<"set_whitebalance_succeed " <<endl;
            assert_success(status);
        }else{
            cout <<"whitebalance_SELECT_fail:  " << status  <<endl;
        }
        status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, WHITEBALANCE);
        if (status == GX_STATUS_SUCCESS){
            cout <<"set_whitebalance_succeed_ggg " <<endl;
            assert_success(status);
        }else{
            cout <<"whitebalance fail:  "  << status <<endl;
        }

#endif


        //设置触发开关为OFF
        status = GXSetEnum(hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
        assert_success(status);

        //注册图像处理回调函数

        status = GXRegisterCaptureCallback(hDevice, NULL, OnFrameCallbackFun);
        cout << "回调函数:" << status << endl;
        //发送开采命令
        GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
        while(1)
        {
//            cout<<"111"<<endl;
            if(detector->key == 'c')
            {
                break;
            }
        };

        // 发送停采命令
        status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);
        // 注销采集回调
        status = GXUnregisterCaptureCallback(hDevice);
    }
    // 在结束的时候调用GXCLoseLib()释放资源
    status = GXCloseDevice(hDevice);
    status = GXCloseLib();

}

void ImageConsProd::ImageConsumer()
{
#ifdef COLLECT_VIDEO
VideoWriter vw("/home/seedcake/桌面/shaobing/data/videoTest.avi",VideoWriter::fourcc('M','J','P','G'),25,Size(VIDEO_WIDTH,VIDEO_HEIGHT),true); // 定义写入视频对象
#endif
    port.initSerialPort();
    Energy energy;
    ArmorRect present, former;
    energy.isRed(false);

    Mat frame;

    //TODO: edit by liuhonjie
    // 读取相机参数
    FileStorage fs(cameraParamsfile, FileStorage::READ);
    Mat cam, dis;
    fs["cameraMatrix"]>>cam;
    cout<<"相机内参: "<<endl;
    cout<<cam<<endl;
    fs["distCoeffs"]>>dis;
    cout<<"畸变系数: "<<endl;
    cout<<dis<<endl;
    // 2020.12.25
    fs.release();
    //TODO: edit by liuhonjie
//    string imgname1;
//    int f = 0;
//    string imgname2;
//    int fff = 1000;
    while (1){
        //TODO: edit by liuhongjie
        port.get_Mode(mode, buff_four, shoot_speed, my_color,
                      detector->now_frame.car_yaw_angel,
                      detector->now_frame.car_pitch_angel,
                      detector->now_frame.yaw_angel_speed,
                      detector->now_frame.pitch_angel_speed);
        //TODO: edit by liuhongjie
        while(prdIdx - csmIdx == 0);                                         //线程锁
        data[csmIdx % BUFFER_SIZE].img.copyTo(frame);                         //将Producer线程生产的图赋值给src
        ++csmIdx;                                                           //解锁，生产下一张图
//        cout<<csmIdx<<endl;

        if(frame.empty()){
            continue;
        }
//        imshow("frame", frame);
        if(mode==1)
        {
#ifdef COLLECT_VIDEO
            vw.write(frame);
#endif
//TODO: edit by liuhongji
            detector->pre_frame = detector->now_frame;
//            cout<<1111111<<endl;
            detector->origin_img = frame;
//            cout<<2222222<<endl;
            frame.copyTo(detector->img);
//            cout<<3333333<<endl;
            detector->set_ROI();
//            cout<<44444444<<endl;
//            GaussianBlur(detector->img, detector->img, Size(5,5),0);
//            cout<<55555555<<endl;
            Mat threshold_image;
            threshold_image = detector->get_LightBar();
            vector<Point2f> mask_point;
            mask_point = detector->get_maskpoint(threshold_image);
            recognize_result res;
            detector->recognize_armoured_plate(mask_point, res);
//            cout<<6666666<<endl;
            detector->now_frame.type = res.type;
//            cout<<"res.type: "<<res.type<<endl;
            if(res.type!=NOT_Armoured_Plate)
            {
                detector->now_frame.armoured_plate = res.best_attack_object;
            }
            detector->pnp(cam,dis,res);
//            cout<<7777777<<endl;
            detector->judge_is_spinning();
            detector->deal_with_spark();

            vdata.dis.f = detector->distance;
            vdata.pitch_angle.f = detector->pitch;// + 1.5;
            vdata.yaw_angle.f = detector->yaw;// - 3.5;
            if(detector->pitch_vec.size()>8)
            {
                vdata.isFindTarget = res.type>0?1:0;
            }
            else
            {
                vdata.isFindTarget = 0;
            }
            vdata.buff_change_four=1;
            vdata.anti_top=1;
            vdata.anti_top_change_armor = 1;
            vdata.nearFace = 1;
            vdata.isfindDafu= 1;
//TODO: edit by liuhongjie
////        // 展示检测效果
#ifdef DEBUG_COUT
        temp(res);
        cout<<res.best_attack_object.size<<endl;
        cout<<"#########################################################"<<endl;
#endif
#ifdef COUT_TRANSFORM_DATA
//    cout<<detector->key<<endl;
    cout<<vdata.dis.f<<" "<<vdata.pitch_angle.f<<" "<<vdata.yaw_angle.f<<endl;
#endif
#ifdef SHOW_IMAGE
        detector->mask_armoured_plate(res);
        detector->show_img(threshold_image, vdata);
#endif
        res.clear();
//TODO: edit by liuhongjie

//            char key = waitKey(1);
//            if(key =='q')
//            {
//                imgname1 = to_string(f++)+".jpg";
//                imwrite(imgname1,num_img);
//                imgname2 = to_string(fff++)+".jpg";
//                imwrite(imgname2,armordetection.img_gramm);

//            }

          }

           if(mode==2)
           {
               double val = 2000;
               cout << "val = " << val << endl;
               GX_STATUS status = GX_STATUS_SUCCESS;
               status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, val);
               int THRESH_BR = 57;//56
               int THRESH_GRAY = 128;//128
               Mat mask;
               energy.isBig = 0;
               energy.getTime = 20;// small:20, big
               energy.setThresh(THRESH_BR, THRESH_GRAY);
               energy.videoProcess(frame, mask, present, former);
               energy.pre_x = present.center.x;
               energy.pre_y = present.center.y;
               //energy.updateSpeed(present, former, frame);
               //energy.predict(present, frame);
//               vdata.pitch_angle.f = 1000;
//               vdata.yaw_angle.f = 1000;
               vdata.pitch_angle.f = energy.pre_y;
               vdata.yaw_angle.f = energy.pre_x;
               vdata.isfindDafu =energy.is_find;
               //vdata.anti_top_change_armor = energy.time2fire;
               vdata.anti_top_change_armor = 1;
               namedWindow("Dafu");
               namedWindow("DafuBinary");
               imshow("Dafu", frame);
               imshow("DafuBinary", mask);
               waitKey(1);
           }

           if(detector->key == 'c')
           {
               break;
           }
           port.TransformData(vdata);
           port.send();
//        //这里记录前10帧的roi信息
//        if ( last_roi.size() >= 10)
//        {
//            last_roi.pop();
//        }
//        last_roi.push(armordetection._next_roi);

//        // 计算掉帧系数
//        if(armordetection._flag == 1)
//        {
//            loss_frame = 0;
//        }
//        else
//        {
//            loss_frame++;
//        }

//        frame_info now_frame_information;
//        now_frame_information.flag =  armordetection._flag;
//        now_frame_information.armor_rect =  armordetection._rotaterect;
//        now_frame_information.roi_rect =  armordetection._now_roi;
//        now_frame_information.tVec =  armordetection._tVec;
//        now_frame_information.delta_T =  armordetection._dt;
////        now_frame_information.carhead_angel_speed =     ;
//        kalman_result = object_predict(frame_information.back(), now_frame_information, 1);
//        kalman_result.at<double>(0); // 上下
//        kalman_result.at<double>(1); // 左右
//        //这里记录前2帧的frame信息
//        if ( frame_information.size() >= 2)
//        {
//            frame_information.pop();
//        }
//        frame_information.push(now_frame_information);
    }
#ifdef COLLECT_VIDEO
    vw.release();
#endif
}


