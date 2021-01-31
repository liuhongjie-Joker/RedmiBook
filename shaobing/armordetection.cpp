#include "opencv2/opencv.hpp"
#include <bits/stdc++.h>
#include "armordetection.h"
#include "mode_define.h"
#include "rp_kalman.h"
#include <vector>
using namespace cv;
using namespace std;
//***********************时间同步器类方法实现***********************//
// 构造函数
time_synchronous::time_synchronous() {
    start = 0.0;
    end = 0.0;
    counter = 0;
}
unsigned int time_synchronous::add_image_collect_time()
{
    double delta = (end - start) / getTickFrequency() * 1000;
    unsigned int res = unsigned int(delta);
}
void time_synchronous::correct(unsigned int value)
{
    counter = value;
}
//***********************帧信息类方法实现***********************//
// 构造函数
frame_information::frame_information()
{
    type = -1;
    delta_T = 0.006;
}
// 获取该帧识别到的装甲板中心
Point2f frame_information::get_armoured_plate_center()
{
    return armoured_plate.center;
}

//***********************识别结果类方法实现***********************//
// 2020.12.25
recognize_result::recognize_result()
{
    type = 0;
}
// 根据识别结果设置该结果的识别类别（0：无；1：小板；2：大板）
void recognize_result::set_type()
{
    if(small.size()>0&&big.size()>0)
    {
        type=BOTH_Armoured_Plate;
    }
    else if(small.size()>0)
    {
        type=SMALL_Armoured_Plate;
    }
    else if(big.size()>0)
    {
        type=BIG_Armoured_Plate;
    }
    else
    {
        type=0;
    }
}
// 找出当前界面最好的识别攻击对象：目前只考虑面积，后面可能会考虑其他因素
void recognize_result::set_best_attack_object(armor_detector *d)
{
    if((ATTACK_PREFERENCE == SMALL_Armoured_Plate || ATTACK_PREFERENCE == BOTH_Armoured_Plate) &&
      (type == SMALL_Armoured_Plate || type == BOTH_Armoured_Plate))
    {
        int maxPosition = max_element(small_score.begin(),small_score.end()) - small_score.begin();
        best_attack_object = small[maxPosition];
        best_attack_object_lightbar_center[0] = small_lightbar_center[maxPosition][0];
        best_attack_object_lightbar_center[1] = small_lightbar_center[maxPosition][1];
    }
    else if((ATTACK_PREFERENCE == BIG_Armoured_Plate || ATTACK_PREFERENCE == BOTH_Armoured_Plate) &&
      (type == BIG_Armoured_Plate || type == BOTH_Armoured_Plate))
    {
        int maxPosition = max_element(big_score.begin(),big_score.end()) - big_score.begin();
        best_attack_object = big[maxPosition];
        best_attack_object_lightbar_center[0] = big_lightbar_center[maxPosition][0];
        best_attack_object_lightbar_center[1] = big_lightbar_center[maxPosition][1];
    }
    if(ATTACK_PREFERENCE == BOTH_Armoured_Plate && type == BOTH_Armoured_Plate)
    {
        // cout<<"1"<<endl;
        int maxPosition1 = max_element(small_score.begin(),small_score.end()) - small_score.begin();
        int maxPosition2 = max_element(big_score.begin(),big_score.end()) - big_score.begin();
        if(big_score[maxPosition2] < small_score[maxPosition1])
        {
            best_attack_object = small[maxPosition1];
            best_attack_object_lightbar_center[0] = small_lightbar_center[maxPosition1][0];
            best_attack_object_lightbar_center[1] = small_lightbar_center[maxPosition1][1];
            type = 1;
        }
        else
        {
            best_attack_object = big[maxPosition1];
            best_attack_object_lightbar_center[0] = big_lightbar_center[maxPosition2][0];
            best_attack_object_lightbar_center[1] = big_lightbar_center[maxPosition2][1];
            type = 2;
        }
    }
    if(d->is_roi == true)
    {
        best_attack_object.center.x += d->roi_lefttop_point.x;
        best_attack_object.center.y += d->roi_lefttop_point.y;
    }
}
// 获取最佳攻击对象的6个点作为pnp测距使用
void recognize_result::get_pnp_point(vector<Point2f> &plist)
{
    Point2f vertices[4];
    best_attack_object.points(vertices);
    for(int i = 0; i < 4; i++)
    {
        for(int j = i + 1; j < 4; j++)
        {
            if(vertices[i].x > vertices[j].x)
            {
                Point2f temp = vertices[i];
                vertices[i] = vertices[j];
                vertices[j] = temp;
            }
        }
    }
    if(vertices[0].y < vertices[1].y)
    {
        Point2f temp = vertices[0];
        vertices[0] = vertices[1];
        vertices[1] = temp;
    }
    if(vertices[2].y < vertices[3].y)
    {
        Point2f temp = vertices[2];
        vertices[2] = vertices[3];
        vertices[3] = temp;
    }
    plist.push_back(vertices[0]);
    plist.push_back((vertices[0]+vertices[1])/2);
    plist.push_back(vertices[1]);
    plist.push_back(vertices[2]);
    plist.push_back((vertices[3]+vertices[2])/2);
    plist.push_back(vertices[3]);
}
// 重置函数
void recognize_result::clear()
{
    type = 0;
    small.clear();
    small_lightbar_center.clear();
    small_score.clear();
    big.clear();
    big_lightbar_center.clear();
    big_score.clear();
    RotatedRect b;
    best_attack_object = b;
}

Point get_center(Rect &rect)
{
    Point cpt;

    cpt.x = rect.x + cvRound(rect.width/2.0);

    cpt.y = rect.y + cvRound(rect.height/2.0);

    return cpt;

}
armor_detector::armor_detector(int width, int height)
{
    key = 's';
    img_width = width;
    img_height = height;
    kf.init_kalman_filter(0);
    pkf.init_kalman_filter(0);
    ykf.init_kalman_filter(0);
}

bool armor_detector::set_safe_ROIRect(Rect &rect_area)
{
    if(rect_area.x < 0)
    {
        rect_area.x = 0;
    }
    if(rect_area.y < 0)
    {
        rect_area.y = 0;
    }
    if(rect_area.x + rect_area.width > img_width)
    {
        rect_area.width = img_width - rect_area.x;
    }
    if(rect_area.y + rect_area.height > img_height)
    {
        rect_area.height = img_height - rect_area.y;
    }
    if(rect_area.width <=0 || rect_area.height <= 0)
    {
        return false;
    }
    return true;
}

void armor_detector::set_ROI()
{
#ifdef COUT_ROI_SIZE
    cout<<"--------------------------------------------------------------------------"<<endl;
    cout<<"pre_frame.type: "<<pre_frame.type<<endl;
    cout<<"pre_frame.armoured_plate.size: "<<pre_frame.armoured_plate.size<<endl;
    cout<<"--------------------------------------------------------------------------"<<endl;
#endif
    is_roi = false;
    roi_lefttop_point.x = 0;
    roi_lefttop_point.y = 0;
    if(pre_frame.type == NOT_Armoured_Plate)
    {
        if(is_loss_object)
        {
            return;
        }
        roi_drop_frame_num += 1;
    }
    is_loss_object = false;
    if(roi_drop_frame_num >= ROI_DROP_FRAME_BOUND)
    {
        roi_drop_frame_num = 0;
        is_loss_object = true;
        return;
    }
    Rect area = pre_frame.armoured_plate.boundingRect();
    Point2f area_center = pre_frame.armoured_plate.center;
    int w = area.width*(1+ROI_WIDTH_EXPAND_ratio*((double)roi_drop_frame_num/ROI_DROP_FRAMENUM_EXPAND + 1));
    w = std::min(w,img_width);
    int h = area.height*(1+ROI_HEIGHT_EXPAND_ratio*((double)roi_drop_frame_num/ROI_DROP_FRAMENUM_EXPAND + 1));
    h = std::min(h,img_height);
    int x = std::max((int)area_center.x - w/2,0);
    int y = std::max((int)area_center.y - h/2,0);
    Rect roi_area(x,y,w,h);
    if(set_safe_ROIRect(roi_area) == false)
    {
        return;
    }
    origin_img(roi_area).copyTo(img);
#ifdef SHOW_IMAGE
    rectangle(origin_img, roi_area, Scalar(255, 255, 255),2, LINE_AA, 0);
#endif
    is_roi = true;
    roi_lefttop_point.x = roi_area.x;
    roi_lefttop_point.y = roi_area.y;
}

// 图片二值化处理
Mat armor_detector::get_LightBar()
{
    Mat result_img, gray;
    Mat gray_binary,tempBinary, complement;
    // 灰度阈值二值
    cvtColor(img,gray,COLOR_BGR2GRAY);
    threshold(gray,gray_binary,GRAY_LOW, GRAY_UP,THRESH_BINARY);
//    erode(gray_binary,gray_binary,getStructuringElement(MORPH_RECT,Size(3,3)));

    // 红蓝通道相减
    vector<Mat> splited;
    split(img,splited);
    if(DETECT_BLUE_LIGHT)
    {
        subtract(splited[0],splited[2],tempBinary);
        threshold(tempBinary,tempBinary,B_SUB_R_LOW, B_SUB_R_UP,THRESH_BINARY);
    }
    else
    {
        subtract(splited[2],splited[0],tempBinary);
        threshold(tempBinary,tempBinary,B_SUB_R_LOW, B_SUB_R_UP,THRESH_BINARY);
    }

    // 补充RGB中间过亮被排除的区域
    if(distance > DISTANCE_BOUND)
    {
        dilate(tempBinary,tempBinary,getStructuringElement(MORPH_RECT,Size(5,5)));
        threshold(gray,complement,230,255,THRESH_BINARY);
        dilate(complement,complement,getStructuringElement(MORPH_RECT,Size(3,3)));
        tempBinary = tempBinary | complement;
    }

    // mask 操作
    result_img = tempBinary & gray_binary;
    return result_img;
}

// 获取匹配灯条上的6个点进行以后的计算
vector<Point2f> armor_detector::get_maskpoint(Mat &result)
{
    //变量声明
    double angel1,angel2,angel_delta;
    double  lightbar_self_ratio = 0;
    vector<Point2f> mask_point;
    vector<Point2f> result_point;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(result, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

    //计算轮廓矩
    vector<Moments> mu(contours.size());
    for (int i = 0; i < (int)contours.size(); i++)
    {
        mu[i] = moments(contours[i], false);
    }
    // 计算轮廓中心, 生成旋转矩形做平行判断
    vector<Point2f>  mc(contours.size());
    for (int i = 0; i < (int)contours.size(); i++)
    {
        mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
    }
    // 保存点
    Point plist[2];
    for (int i = 0; i < (int)contours.size(); i++)
    {
        //保存这个灯条的上中下顶点坐标
        plist[0] = find_minY_point(contours[i]);
        plist[1] = find_maxY_point(contours[i]);
        RotatedRect rect = minAreaRect(contours[i]);
        if(rect.size.width <= 3 || rect.size.height <= 3)
        {
            continue;
        }
//        if(rect.size.width > rect.size.height)
//        {
//            lightbar_self_ratio = rect.size.width/rect.size.height - LIGHTBAR_self_ratio;
//        }
//        else
//        {
//            lightbar_self_ratio = rect.size.height/rect.size.width - LIGHTBAR_self_ratio;
//        }
        lightbar_self_ratio = rect.size.height/rect.size.width - LIGHTBAR_self_ratio;
        if(lightbar_self_ratio > LIGHTBAR_self_ratio_error_bound
                ||  lightbar_self_ratio < -LIGHTBAR_self_ratio_error_bound)
        {
            continue;
        }
        mask_point.push_back(Point2f(plist[0].x,plist[0].y));
        mask_point.push_back(mc[i]);
        mask_point.push_back(Point2f(plist[1].x,plist[1].y));
    }
#ifdef DEBUG_COUT
    cout<<"**********************************************************************"<<endl;
    cout<<"经过灯条自身的长宽比筛选后"<<endl;
    cout<<"剩下的轮廓数："<<mask_point.size()<<endl;
    cout<<"**********************************************************************"<<endl;
#endif

    int size = mask_point.size();

    for(int i = 0; i<size; i+=3)
    {
        for(int j = i + 3; j<size; j+=3)
        {
            // 计算两灯条的角度差
            angel1 = fastAtan2(mask_point[i+2].y-mask_point[i].y,mask_point[i+2].x-mask_point[i].x);
            angel2 = fastAtan2(mask_point[j+2].y-mask_point[j].y,mask_point[j+2].x-mask_point[j].x);
            double temp = max(angel1,angel2);
            angel2 = min(angel1,angel2);
            angel1 = temp;
            if(angel1 > 145 || angel2 < 35)
            {
                continue;
            }
            angel_delta = angel1-angel2;
#ifdef DEBUG_COUT_ANGEL
            cout<<"**********************************************************************"<<endl;
            cout<<"两灯条角度差："<<angel_delta<<endl;
            cout<<"角度差的上界："<<ANGEL_DELTA_error_bound<<endl;
            cout<<"**********************************************************************"<<endl;
#endif
            // 判断两灯条的角度差是否符合误差限
            if(angel_delta > ANGEL_DELTA_error_bound)
            {
                continue;
            }
            for(int k = 0; k<3; k++)
                result_point.push_back(mask_point[i+k]);
            for(int k = 0; k<3; k++)
                result_point.push_back(mask_point[j+k]);
        }
    }
#ifdef DEBUG_COUT
    cout<< "经过灯条角度匹配后，得到可能为装甲板的数量为：" << result_point.size()/3 - 1<<endl;
#endif
    return result_point;
}

// 寻找匹配的两个灯条
void armor_detector::recognize_armoured_plate(vector<Point2f> &mask_point,recognize_result &res)
{
    double center_deltaX,center_deltaY, light_bar_length,delta_x,
            light_bar_length1,light_bar_length2,
            ratio1,ratio2,
            lightbar_ratio,
            area,
            center_slope,
            lightbar_movex,lightbar_slope1, lightbar_slope2;
    RotatedRect rRect;
#ifdef DEBUG_COUT
    cout<<"共有轮廓数量："<<mask_point.size()<<endl;
#endif
    for(int i=0; i< (int)mask_point.size();i+=6)
    {
        vector<Point2f> match_lightbar_center;
        vector<Point2f> compare_point_list;
        for(int k=0; k<6;k++)
        {
            compare_point_list.push_back(mask_point[i+k]);
        }
        match_lightbar_center.push_back(compare_point_list[1]);
        match_lightbar_center.push_back(compare_point_list[4]);
        rRect = minAreaRect(compare_point_list);

        center_deltaX = fabs(compare_point_list[4].x - compare_point_list[1].x);
        center_deltaY = fabs(compare_point_list[4].y - compare_point_list[1].y);
        light_bar_length1 = calculate_oushi_distance(compare_point_list[0], compare_point_list[2]);
        light_bar_length2 = calculate_oushi_distance(compare_point_list[3], compare_point_list[5]);
        light_bar_length = rRect.size.height;

#ifdef DEBUG_COUT
    cout<<"**********************************************************************"<<endl;
    cout<<"中心点Y轴偏移量："<<center_deltaY<<endl;
    cout<<"中心点Y轴偏移量允许的上界："<<CENTER_DELTA_Y_up_bound<<endl;
    cout<<"两者的（前减后）差："<<center_deltaY-CENTER_DELTA_Y_up_bound<<endl;
    cout<<"**********************************************************************"<<endl;
#endif
        if(center_deltaY>CENTER_DELTA_Y_up_bound)
        {
            continue;
        }

#ifdef DEBUG_COUT
    cout<<"**********************************************************************"<<endl;
    cout<<"两个中心点X轴的距离要大于两个灯条的长度"<<endl;
    cout<<"两个中心点X轴的距离："<<center_deltaX<<endl;
    cout<<"灯条1的长度："<<light_bar_length1<<endl;
    cout<<"灯条2的长度："<<light_bar_length2<<endl;
    cout<<"**********************************************************************"<<endl;
#endif
        // 框必须比长大的矩形才有可能是装甲板
        if(center_deltaX < light_bar_length1 || center_deltaX < light_bar_length2)
        {
            continue;
        }

        // 计算灯条比值 - 1
        lightbar_ratio = fabs(light_bar_length1/light_bar_length2 - 1);
#ifdef DEBUG_COUT
    cout<<"**********************************************************************"<<endl;
    cout<<"两灯条长度的比值要尽可能接近1"<<endl;
    cout<<"两灯条长度的比值与1的差值："<<lightbar_ratio<<endl;
    cout<<"上述差值允许的上界："<<DELTA_LIGHTBAR_ratio_error_bound<<endl;
    cout<<"**********************************************************************"<<endl;
#endif
        if(lightbar_ratio>DELTA_LIGHTBAR_ratio_error_bound)
        {
            continue;
        }

        // 判断两中心点所成直线的斜率是否符合要求
        center_slope = fabs(compare_point_list[1].y - compare_point_list[4].y) / center_deltaX;
#ifdef DEBUG_COUT
    cout<<"**********************************************************************"<<endl;
    cout<<"两中心点所成直线的斜率要尽可能接近0"<<endl;
    cout<<"两中心点所成直线的斜率："<<center_slope<<endl;
    cout<<"上述斜率允许的上界："<<CENTER_SLOPE_error_bound<<endl;
    cout<<"**********************************************************************"<<endl;
#endif
        if(center_slope>CENTER_SLOPE_error_bound)
        {
            continue;
        }

        // 考虑识别出的两个灯条长度不完全相同，计算出两个比值进行判断，避免疏漏
        delta_x = rRect.size.width;
        // 判断矩形面积是否符合条件
        area = delta_x*light_bar_length;
        if(area<MIN_RECTANGLE_SIZE)
        {
            continue;
        }

        ratio1 = center_deltaX/light_bar_length1 - RECOGNIZE_Small_Param1;
        ratio2 = center_deltaX/light_bar_length2 - RECOGNIZE_Small_Param1;
#ifdef DEBUG_COUT
    cout<<"**********************************************************************"<<endl;
    cout<<"装甲板长宽比尽可能符合实际"<<endl;
    cout<<"灯条1的长宽比与实际小装甲板的差值："<<ratio1<<endl;
    cout<<"灯条2的长宽比与实际小装甲板的差值："<<ratio2<<endl;
    cout<<"小装甲板长宽比的误差限："<<PARAM1_error_up_bound<<endl;
    cout<<"小装甲板长宽比的误差限："<<PARAM1_error_low_bound<<endl;
#endif
        // 查看长宽比是否符合条件
        if((ratio1<PARAM1_error_up_bound && ratio1 > PARAM1_error_low_bound)&&
                (ratio2<PARAM1_error_up_bound && ratio2 > PARAM1_error_low_bound))
        {
            res.small.push_back(rRect);
            res.small_score.push_back(area);
            res.small_lightbar_center.push_back(match_lightbar_center);
            continue;
        }

        // 大装甲板
        // 考虑识别出的两个灯条长度不完全相同，计算出两个比值进行判断，避免疏漏
        ratio1 = center_deltaX/light_bar_length1 - RECOGNIZE_Big_Param1;
        ratio2 = center_deltaX/light_bar_length2 - RECOGNIZE_Big_Param1;
#ifdef DEBUG_COUT
    cout<<"灯条1的长宽比与实际大装甲板的差值："<<ratio1<<endl;
    cout<<"灯条2的长宽比与实际大装甲板的差值："<<ratio2<<endl;
    cout<<"大装甲板长宽比的误差限："<<PARAM2_error_up_bound<<endl;
    cout<<"大装甲板长宽比的误差限："<<PARAM2_error_low_bound<<endl;
    cout<<"**********************************************************************"<<endl;
#endif
        // 查看长宽比是否符合条件
        if((ratio1<PARAM2_error_up_bound && ratio1 >PARAM2_error_low_bound)
           &&(ratio2<PARAM2_error_up_bound && ratio2 >PARAM2_error_low_bound))
        {

            res.big.push_back(rRect);
            res.big_score.push_back(area);
            res.big_lightbar_center.push_back(match_lightbar_center);
        }
    }
    res.set_type();
//    cout<<"res.type2: "<<res.type<<endl;
    res.set_best_attack_object(this);
}

// 标记出最好的装甲板
void armor_detector::mask_armoured_plate(recognize_result &res)
{
    cv::Point2f* vertices = new cv::Point2f[4];
    res.best_attack_object.points(vertices);
    for (int i = 0; i < 4; i++)
    {
        cv::line(origin_img, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 4, 8, 0);
    }
}

//pnp
void armor_detector::pnp(Mat &cameraMatrix, Mat &distCoeffs, recognize_result &res)
{
    Mat rvec, tvec;
    if(res.type == SMALL_Armoured_Plate)
    {
        vector<Point3f> obj=vector<Point3f>
        {
                Point3f(-SMALL_Armoured_Plate_width/2, Light_Bar_Length/2, 0),
                Point3f(-SMALL_Armoured_Plate_width/2, 0, 0),
                Point3f(-SMALL_Armoured_Plate_width/2, -Light_Bar_Length/2, 0),
                Point3f(SMALL_Armoured_Plate_width/2, Light_Bar_Length/2, 0),
                Point3f(SMALL_Armoured_Plate_width/2, 0, 0),
                Point3f(SMALL_Armoured_Plate_width/2, -Light_Bar_Length/2, 0),
        };
        vector<Point2f> best_mask_point;
        res.get_pnp_point(best_mask_point);
//            for(int i = 0; i<best_mask_point.size(); i++)
//            {
//                cout<<best_mask_point[i]<<endl;
//            }
        solvePnP(obj,best_mask_point,
                     cameraMatrix,distCoeffs,
                     rvec,tvec,
                     0,SOLVEPNP_ITERATIVE);
        now_frame.tVec = tvec;
    }
    else if(res.type == BIG_Armoured_Plate)
    {
        vector<Point3f> obj=vector<Point3f>
        {
                Point3f(-BIG_Armoured_Plate_width/2, Light_Bar_Length/2, 0),
                Point3f(-BIG_Armoured_Plate_width/2, 0, 0),
                Point3f(-BIG_Armoured_Plate_width/2, -Light_Bar_Length/2, 0),
                Point3f(BIG_Armoured_Plate_width/2, Light_Bar_Length/2, 0),
                Point3f(BIG_Armoured_Plate_width/2, 0, 0),
                Point3f(BIG_Armoured_Plate_width/2, -Light_Bar_Length/2, 0),
        };
        vector<Point2f> best_mask_point;
        res.get_pnp_point(best_mask_point);
//            for(int i = 0; i<best_mask_point.size(); i++)
//            {
//                cout<<best_mask_point[i]<<endl;
//            }
        solvePnP(obj,best_mask_point,
                     cameraMatrix,distCoeffs,
                     rvec,tvec,
                     0,SOLVEPNP_ITERATIVE);
        now_frame.tVec = tvec;
    }
    if(res.type != NOT_Armoured_Plate)
    {
        distance = sqrt(pow(tvec.at<double>(0),2)+pow(tvec.at<double>(1),2)+pow(tvec.at<double>(2),2))*10;
        distance = kf.correct_value(distance);
        pitch = atan2(-tvec.at<double>(1), tvec.at<double>(2))*180/pi;
//        pitch = pkf.correct_value(pitch);
        yaw = atan2(tvec.at<double>(2), tvec.at<double>(0))*180/pi - 90;
//        yaw = ykf.correct_value(yaw);
    }
    else
    {
        distance = 0;
        pitch = 0;
        yaw = 0;
        kf.init_kalman_filter(0);
        pkf.init_kalman_filter(0);
        ykf.init_kalman_filter(0);
    }
}

void armor_detector::put_txt_to_img(VisionData &vdata)
{
    //设置绘制文本的相r关参数
    string temp;
    string text1 = "distance: "+ to_string(distance);
    string text2 = "send_pitch: " + to_string(vdata.pitch_angle.f);
    string text3 = "send_yaw: " + to_string(vdata.yaw_angle.f);
    if(now_frame.type == 1)
    {
        temp = "small armor!";
    }
    else if(now_frame.type == 2)
    {
        temp = "big armor!";
    }
    else
    {
        temp = "nothing!";
    }
    string text4 = "isFindTarget: " + temp;
    int font_face = FONT_HERSHEY_COMPLEX;
    double font_scale = 1.2;
    int thickness = 2;
    int baseline;
    //获取文本框的长宽
    Size text1_size = getTextSize(text1, font_face, font_scale, thickness, &baseline);
    //将文本框居中绘制
    Point origin;
    origin.x = 20;
    origin.y = text1_size.height*1;
    putText(origin_img, text1, origin, font_face, font_scale, Scalar(255, 255, 255), thickness, 8, 0);
    origin.y = text1_size.height * 3;
    putText(origin_img, text2, origin, font_face, font_scale, Scalar(255, 255, 255), thickness, 8, 0);
    origin.y = text1_size.height * 5;
    putText(origin_img, text3, origin, font_face, font_scale, Scalar(255, 255, 255), thickness, 8, 0);
    origin.y = text1_size.height * 7;
    putText(origin_img, text4, origin, font_face, font_scale, Scalar(0, 255, 0), thickness, 8, 0);
    if(is_spinning == true)
    {
        //设置绘制文本的相关参数

        origin.y = text1_size.height * 9;
        string text5 = "The car is spinning!";
        putText(origin_img, text5, origin, font_face, font_scale, Scalar(0, 255, 0), thickness, 8, 0);
        temp = "rotate to left";
        if(spin_direction == -2)
        {
            temp = "rotate to right";
        }
        origin.y = text1_size.height * 11;
        string text6 = "Spin_Direction: " + temp;
        putText(origin_img, text6, origin, font_face, font_scale, Scalar(0, 255, 0), thickness, 8, 0);
    }
}

void armor_detector::show_img(Mat &threshold_image, VisionData &vdata)
{
    if(key != 'q')
    {
        put_txt_to_img(vdata);
        //namedWindow(WINDOW_NAME, WINDOW_NORMAL);
        imshow("roi_img", img);
        imshow("threshold_image", threshold_image);
        imshow(WINDOW_NAME, origin_img);
        waitKey(0);
        char temp = waitKey(1);
        if(temp == 'q' || temp == 'c')
        {
            key = temp;
            destroyAllWindows();
        }
    }
//    cout<<key<<endl;
}

// 寻找各个灯条上中下点的函数集合
double calculate_oushi_distance(Point2f &a1,Point2f &a2)
{
    return sqrt((a2.x-a1.x)*(a2.x-a1.x)+(a2.y-a1.y)*(a2.y-a1.y));
}
Point find_maxY_point(vector<Point> &plist)
{
    double temp=0;
    Point maxY_point;
    for(int i=0;i<(int)plist.size();i++)
    {
        if(plist[i].y>temp)
        {
            temp = plist[i].y;
            maxY_point=plist[i];
        }
    }
    return maxY_point;
}
Point find_minY_point(vector<Point> &plist)
{
    double temp=10000;
    Point minY_point;
    for(int i=0;i<(int)plist.size();i++)
    {
        if(plist[i].y<temp)
        {
            temp = plist[i].y;
            minY_point=plist[i];
        }
    }
    return minY_point;
}
Point find_maxX_point(vector<Point> &plist)
{
    double temp=0;
    Point maxX_point;
    for(int i=0;i<(int)plist.size();i++)
    {
        if(plist[i].x>temp)
        {
            temp = plist[i].x;
            maxX_point=plist[i];
        }
    }
    return maxX_point;
}
Point find_minX_point(vector<Point> &plist)
{
    double temp=10000;
    Point minX_point;
    for(int i=0;i<(int)plist.size();i++)
    {
        if(plist[i].x<temp)
        {
            temp = plist[i].x;
            minX_point=plist[i];
        }
    }
    return minX_point;
}


void temp(recognize_result &res)
{
    if(res.type == NOT_Armoured_Plate)
    {
        cout<<"界面没有出现装甲板。"<<endl;
    }
    else if(res.type == SMALL_Armoured_Plate)
    {
        cout<<"界面出现小装甲板了！小小小小小小小小小小"<<endl;
    }
    else if(res.type == BIG_Armoured_Plate)
    {
        cout<<"界面出现大装甲板了！大大大大大大大大大大"<<endl;
    }
    else if(res.type == BOTH_Armoured_Plate)
    {
        cout<<"界面同时出现大小装甲板了！同时出现同时出现同时出现"<<endl;
    }
}

/************* 闪烁掉帧处理有关的函数 ***************/
// 用vector模拟队列进出
void insert_new_value(vector<double> &vec, double value)
{
    if(vec.size() < QUEUE_MAX)
    {
        vec.push_back(value);
    }
    else
    {
        vector<double>::iterator first = vec.begin();
        vec.erase(first);//删除第一个元素
        vec.push_back(value);
    }
}
// 计算向量列表平均值
double get_vector_average(vector<double> &vec)
{
    double sum = accumulate(std::begin(vec), std::end(vec), 0.0);
    return sum / vec.size(); //均值
}
// 计算向量差分平均值
double get_delta_average(vector<double> &vec)
{
    double average_delta=0.0;
    if(vec.size()>=2)
    {
        average_delta = (vec[vec.size()-1] - vec[0])/((double)vec.size()-1.0);
    }
    return average_delta; //均值
}
// 预测闪烁掉帧后的数值变化：flag=0表示pitch,flag=1表示yaw
double armor_detector::predict_spark(double &angle, int flag)
{
    if(flag==0)
    {
        if(pitch_vec.size()>0)
        {
            angle = pitch_vec[pitch_vec.size()-1]+get_delta_average(pitch_vec)*drop_frame_times;
        }
    }
    else
    {
        if(yaw_vec.size()>0)
        {
            angle = yaw_vec[yaw_vec.size()-1]+get_delta_average(yaw_vec)*drop_frame_times;
        }
    }
}
// 闪烁掉帧处理有关的函数
void armor_detector::deal_with_spark()
{
    if(now_frame.type != NOT_Armoured_Plate)
    {
        drop_frame_times = 0;
        insert_new_value(dis_vec, distance);
        insert_new_value(yaw_vec, yaw);
        insert_new_value(pitch_vec, pitch);
        return;
    }
    if(drop_frame_times<LOSS_OBJECT)
    {
        drop_frame_times++;
        predict_spark(pitch,0);
        predict_spark(yaw,1);
    }
    else
    {
        yaw_vec.clear();
        pitch_vec.clear();
    }
}
// 识别反陀螺函数集合
void armor_detector::judge_is_spinning()
{
    if(now_frame.type == NOT_Armoured_Plate)
    {
        drop_frame_param++;
        if(drop_frame_param>DROP_FRAME_PARAM_BOUND)
        {
            is_spinning = false;
            spinning_param = 0;
        }
        return;
    }
    drop_frame_param = 0;
    if(pre_frame.type == NOT_Armoured_Plate)
    {
        // 计算掉帧前的物距
        double pre_dis = 0;
        if(dis_vec.size()>0)
        {
            pre_dis = dis_vec[dis_vec.size() - 1];
        }
        // 计算物体掉帧前后的位移，一般陀螺状态下，这段位移不是很大，但是会相对确定。
        double move_distance = fabs(distance - pre_dis);
#ifdef DEBUG_SPIN_COUT
        cout<<"*********************************************"<<endl;
        cout<<"物体掉帧前后的位移: "<< move_distance <<endl;
        cout<<"识别为陀螺状态的位移误差上限: "<< SPINNING_DISTENCE_UP_BOUND <<endl;
        cout<<"识别为陀螺状态的位移误差下限: "<< SPINNING_DISTENCE_LOW_BOUND <<endl;
#endif
/*
下一个条件是为了将陀螺状态与闪烁掉帧情况彻底区分开来
通过yaw轴差分，锁定陀螺方向，因为yaw轴的差分在转动到另一个装甲板时是异号。
而闪烁掉帧情况下，如果非陀螺仪状态，那么yaw轴的差分一般都是同号的。
一般认为陀螺方向一段时间内是不变的。
要变陀螺方向的话，需要减速到停，这个过程够退出陀螺状态的识别。
因此，进行新方向的陀螺测试时，前面的测试参数已经清空，不会与上一次冲突。
*/
        //
        double pre_yaw_delta = 0, yaw_delta = 0;
        if(yaw_vec.size()>=2)
        {
            pre_yaw_delta = yaw_vec[yaw_vec.size() - 1] - yaw_vec[yaw_vec.size() - 2];
        }
        if(yaw_vec.size()>0)
        {
            yaw_delta = yaw - yaw_vec[yaw_vec.size() - 1];
        }
        double now_spin_direction = MathUtils_SignBit(pre_yaw_delta) - MathUtils_SignBit(yaw_delta);
#ifdef DEBUG_SPIN_COUT
        cout<<"now_spin_direction = "<< now_spin_direction<<endl;
        cout<<"*********************************************"<<endl;
#endif
        if(spin_direction == 0)
        {
            spin_direction = now_spin_direction;
        }
        if(move_distance>SPINNING_DISTENCE_LOW_BOUND &&
           move_distance<SPINNING_DISTENCE_UP_BOUND&&
           MathUtils_SignBit(pre_yaw_delta) != MathUtils_SignBit(yaw_delta))
        {
            if(now_spin_direction != spin_direction)
            {
                spin_direction = now_spin_direction;
            }
            spinning_param++;
            continues_judge_param = 0;
#ifdef DEBUG_SPIN_COUT
        cout<<"陀螺系数加一了!"<<endl;
#endif
        }
        else
        {
            // spinning_param--;
            continues_judge_param++;
        }
#ifdef DEBUG_SPIN_COUT
        cout<<"is_spinning = "<<is_spinning<<endl;
        cout<<"||||||||||||||||||||||||||||||"<<endl;
#endif
    }
    else
    {
        continues_judge_param++;
    }
    if(spinning_param >= SPINNING_BOUND)
    {
        is_spinning = true;
    }
    if(continues_judge_param > CONTINUES_JUDGE_PARAM_BOUND || drop_frame_param > DROP_FRAME_PARAM_BOUND)
    {
        spinning_param = 0;
        spin_direction = 0;
        is_spinning = false;
    }
}
