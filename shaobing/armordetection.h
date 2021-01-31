#include "opencv2/opencv.hpp"
#include <bits/stdc++.h>
#include <serialport.h>
#include "rp_kalman.h"
using namespace cv;
using namespace std;
#ifndef ARMOUR_DETECTOR_H
#define ARMOUR_DETECTOR_H


#define pi 3.14159265
#define MathUtils_SignBit(x) \
    (((signed char*) &x)[sizeof(x) - 1] >> 7 | 1)
#define WINDOW_NAME "result"
// 灰度阈值
#define GRAY_LOW 50
#define GRAY_UP 255

// 红蓝阈值
#define B_SUB_R_LOW 60
#define B_SUB_R_UP 255
#define R_SUB_B_LOW 60
#define R_SUB_B_UP 255

// 与PnP测距有关的参量
#define cameraParamsfile "/home/nuc/桌面/cameraParams.xml"
//#define cameraParamsfile "/home/seedcake/桌面/cameraParams.xml"

// 与装甲板识别有关的阈值
#define NOT_Armoured_Plate 0
#define SMALL_Armoured_Plate 1
#define BIG_Armoured_Plate 2
#define BOTH_Armoured_Plate 3
// 注意：这里并非装甲板的真实宽度，而是两个灯条的中心距
// 小装甲板
#define SMALL_Armoured_Plate_width 13.0
#define Light_Bar_Length 5.55
// 前面两者的比值
#define RECOGNIZE_Small_Param1 2.342342

// 大装甲板
#define BIG_Armoured_Plate_width 22.85
#define Light_Bar_Length 5.55
// 前面两者的比值
#define RECOGNIZE_Big_Param1 4.117117

#define PARAM1_error_up_bound 0.5
#define PARAM1_error_low_bound -0.5
#define PARAM2_error_up_bound 1.1
#define PARAM2_error_low_bound -1.1

// 灯条自身的宽
#define Light_Bar_WIDTH 0.75
// 灯条自身的长宽比
#define LIGHTBAR_self_ratio 0.25
// 灯条自身的长宽比误差限
#define LIGHTBAR_self_ratio_error_bound 8
// 灯条长度比值误差限
#define DELTA_LIGHTBAR_ratio_error_bound 0.5
// 矩形框面积阈值
#define MIN_RECTANGLE_SIZE 100
// 中心点允许的斜率范围
#define CENTER_SLOPE_error_bound 0.15
#define CENTER_DELTA_Y_up_bound 120
// 两个灯条的角度差的误差限
#define ANGEL_DELTA_error_bound 15

// 两灯条横向偏移量的比值与1的差值的误差限
#define LIGHTBAR_MOVEX_up_bound 4
#define LIGHTBAR_MOVEX_low_bound -4

// 轮廓上限
#define CONTOURS_BOUND 15

// 旋转矩形角度差
#define RotatedRect_ANGEL_DELTA_up_bound 15

// 攻击倾向，侧重攻击大的还是小的
#define ATTACK_PREFERENCE 3

// 与反陀螺有关的阈值
#define SPINNING_BOUND 8
#define CONTINUES_JUDGE_PARAM_BOUND 60
#define DROP_FRAME_PARAM_BOUND 30
#define SPINNING_DISTENCE_UP_BOUND 350.0
#define SPINNING_DISTENCE_LOW_BOUND 25.0

// 与ROI有关的参数设置
#define ROI_DROP_FRAME_BOUND 3
#define ROI_DROP_FRAMENUM_EXPAND 1
#define ROI_WIDTH_EXPAND_ratio 1
#define ROI_HEIGHT_EXPAND_ratio 3

// 二值化处理换策略时的距离上界
#define DISTANCE_BOUND 2000

// 闪烁掉帧处理有关的参数
#define QUEUE_MAX 20
#define LOSS_OBJECT 30
#define SPARK_ALPHA 1
#define SPARK_BELTA 1

struct armor_detector;
struct frame_information;
struct recognize_result;
struct frame_information
{
    int type;
    RotatedRect armoured_plate;
    Mat tVec;
    double pitch;
    double yaw;
    double delta_T;
    double object_speed;
    double pitch_speed;
    double yaw_speed;


    double car_yaw_angel;
    double car_pitch_angel;
    double yaw_angel_speed;
    double pitch_angel_speed;
    double carhead_angel_speed;
    frame_information();
    Point2f get_armoured_plate_center();
};
struct recognize_result{
    int type;
    vector<RotatedRect> small;
    vector< vector<Point2f>> small_lightbar_center;
    vector<double> small_score;
    vector<RotatedRect> big;
    vector<double> big_score;
    vector< vector<Point2f>> big_lightbar_center;
    RotatedRect best_attack_object;
    Point2f best_attack_object_lightbar_center[2];
    recognize_result();
    void set_type();
    void set_best_attack_object(armor_detector *detector);
    void get_pnp_point(vector<Point2f> &plist);
    void clear();
};
// 时间同步器
struct time_synchronous {
    double start;
    double end
    unsigned int counter;
    time_synchronous();
    void add_image_collect_time();
    void correct(unsigned int value);
};

struct armor_detector{
    Mat img;
    Mat origin_img;
    int img_width;
    int img_height;
    char key;
    // 与ROI有关的参数设置
    int roi_drop_frame_num = 0;


    // 与反陀螺有关的变量
    bool is_spinning = false;
    int spinning_param = 0;
    int continues_judge_param = 0;
    int drop_frame_param = 0;

    // 与闪烁掉帧处理有关的变量
    int drop_frame_times = 0;
    // 用于判断陀螺是向视角左转还是右转，左为-2，右为2
    int spin_direction = 1;
    vector<double> yaw_vec;
    vector<double> pitch_vec;
    vector<double> dis_vec;

    // 前后帧信息变量
    frame_information pre_frame;
    frame_information now_frame;

    // 设置ROI区域
    bool is_roi = false;
    bool is_loss_object = false;
    Point2f roi_lefttop_point;

    kalman_filter kf;
    kalman_filter pkf;
    kalman_filter ykf;

    double distance;
    double pitch;
    double yaw;

    // 时间同步系统
    time_synchronous ts;
    armor_detector(int width, int height);
    // 装甲板识别有关的函数
    Mat get_LightBar();
    bool set_safe_ROIRect(Rect &rect_area);
    void set_ROI();
    vector<Point2f> get_maskpoint(Mat &result);
    void recognize_armoured_plate(vector<Point2f> &mask_point, recognize_result &res);
    void pnp(Mat &cameraMatrix, Mat &distCoeffs, recognize_result &res);
    // 识别陀螺状态的函数
    void judge_is_spinning();
    // 闪烁掉帧处理有关的函数
    double predict_spark(double &angle, int flag);
    void deal_with_spark();
    // 与显示图片有关的函数
    void put_txt_to_img(VisionData &vdata);
    void mask_armoured_plate(recognize_result &res);
    void show_img(Mat &threshold_image, VisionData &vdata);
};
double calculate_oushi_distance(Point2f &a1,Point2f &a2);
Point get_center(Rect &rect);
Point find_maxY_point(vector<Point> &plist);
Point find_minY_point(vector<Point> &plist);
Point find_maxX_point(vector<Point> &plist);
Point find_minX_point(vector<Point> &plist);
void temp(recognize_result &res);
#endif // ARMOUR_DETECTOR_H
