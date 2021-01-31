#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <rp_kalman.h>
using namespace cv;//���Ǹ����õı��ϰ�ߣ���������cv::

extern RotatedRect center_R;
extern const double PI;
extern int SCREEN_MAX_X;//��Ļ�ֱ���1280x720
extern int SCREEN_MAX_Y;

class RMvector
{
	// ���Ż�
	// ���Ӧ���ǿ��Ըĳ�Point2f�������࣬����뷨�е�Ƿ�ף�xy��ͻ
public:
	// û���õ������յ����Ϣ
	//Point2f start_point = Point2f(0, 0);// ��ʼ��
	//Point2f end_point = Point2f(0, 0);	// ��ֹ��
	//Point2f vec;						// ��ԭ��Ϊ���ĵ�����
	double x;							// ��ԭ��Ϊ���ĵ�x����
	double y;							// ��ԭ��Ϊ���ĵ�y����
	double angle;						// �����ĽǶȣ��Ժ���Ϊ��׼
	RMvector()
	{
		//start_point = Point2f(0, 0);
		//end_point = Point2f(0, 0);
		x = 0; y = 0;
		angle = 0;
	}
	RMvector(Point2f& start, Point2f& end)
	{
		//start_point = start;
		//end_point = end;
		//Point2f vec = end_point - start_point;
		Point2f vec = end - start;
		x = vec.x; y = vec.y;
		angle = atan(y / x);// ���Ŀ�����Դ
	}
	RMvector(double a)// �ɽǶȼ��㵥λ����
	{
		x = cos(a * PI / 180); y = sin(a * PI / 180);
		angle = a; // ���ܳ���0-360��Χ������ȫ�������Ϊmod
	}
	double getNorm()// ���������ĳ���
	{
		return sqrt(x * x + y * y);
	}
	double dot(RMvector& new_vector)// �����������һ������
	{
		return x * new_vector.x + y * new_vector.y;
	}
	double cross(RMvector& new_vector)// �����������һ��������ע��˳��
	{
		return x* new_vector.y - y * new_vector.x;
	}
};

//class RCenter: public RotatedRect // �̳���ת����
//{
//	// ����̳�ò�Ʋ�û��ʲô���ã����ˣ����̳���
//	// Բ����ʲô����ĺ�����
//	// ��ҪΪ�˴����װ��д����ҪΪ�˴���������ԣ������ݲ�����
//};

class ArmorRect
{
	// ������Ըĳ���ת���ε�������
public:
	Point2f center;					// ��ת���ε�����
	double width;					// ��ת���ε�width
	double height;					// ��ת���ε�height
	double long_edge;				// ����
	double short_edge;				// �̱�
	double rotatedrect_angle;		// ��ת����ԭ�еĽǶ�(-90, 0]
	double angle;					// װ�װ�ĽǶȣ�[0, 180);	//���������ǶȾͿ��Լ������ߺͷ��ߵ�������
	double tangent_angle;			// װ�װ����߽Ƕ�
	double area;					// ���
	RMvector to_center;				// ��������
	RMvector tangent;				// ����
	double speed;					// װ�װ�Ľ��ٶȣ�ÿ֡����rad
	//RotatedRect rotRect;
	bool hitted;					// ��װ�װ��Ƿ񱻵�����־λ // �ⲿ�ӿ�
	RMvector vertex2center[4];
	Point2f vertices[4];
	double eps = 0.2;				// �ٶ���ֵ�� С�������ֵ�ٶ���Ϊ0
    int minErrorIndex = -1;
	ArmorRect() {};
	ArmorRect(RotatedRect& rect)
	{
		//rotRect = rect;
		center = rect.center;
		width = rect.size.width;
		height = rect.size.height;
		long_edge = max(width, height);
		short_edge = min(width, height);
		rotatedrect_angle = rect.angle;
		area = width * height;
		setAngel();
		setTangent();
		//RMvector vec(center, center_R.center);// �����������()��������򻯴��룬һ��С�Ż������Ǳ�Ҫ
		//to_center = vec;
		speed = 0;
		hitted = 0;
		rect.points(vertices);
	}
	void setParam(RotatedRect& rect)// ���ڸ����µ�װ�װ�
	{
		//rotRect = rect;
		center = rect.center;
		width = rect.size.width;
		height = rect.size.height;
		long_edge = max(width, height);
		short_edge = min(width, height);
		rotatedrect_angle = rect.angle;
		area = width * height;
		setAngel(); // ���ýǶ�
		setTangent();
		//RMvector vec(center, center_R.center);// �����������()��������򻯴���
		//to_center = vec;
		speed = 0;
		hitted = 0;
		rect.points(vertices);
	}
	void setSpeed(double w)
	{
		if (w < eps) // ���˳���жϴ�������
			speed = 0;
		speed = w;
		// ��Ҫ�����µ��زĽ��в���
		// std::cout << "armorSpeed" <<speed << std::endl;
	}
	void setAngel()
	{
		// ����
		angle = width > height ? (rotatedrect_angle + 90) :  rotatedrect_angle;
		//std::cout << "angle: " << angle << std::endl;
	}
	void setTangent()
	{
		// ����
		tangent_angle = width < height ? (rotatedrect_angle + 90) : rotatedrect_angle;
		//std::cout << "tangent: " << tangent_angle << std::endl;
	}
	void setToCenter()
	{		
		for (int i = 0; i < 4; i++)
		{
			RMvector vec1(vertices[i], center_R.center);
			vertex2center[i] = vec1; // ����ֱ�Ӹ�ֵ��ֵ����
		}
		RMvector vec(center, center_R.center);// �����������()��������򻯴��룬һ��С�Ż������Ǳ�Ҫ
		to_center = vec;
	}
};


typedef struct Roi
{
	Point center;
	int width;
	int height;
	Point Anchor;		// ROI ֱ�����ε�rect��xy
	double scale = 1.5;		// ��ʱΪ1������ԭ״
	void getAnchor()
	{
		Anchor.x = (int)center.x - width / 2;
		Anchor.y = (int)center.y - height / 2;
	}
	Roi()// �޲ι��캯�� // ���ڹ���������Ļ��ROI
	{
		center = Point(SCREEN_MAX_X / 2, SCREEN_MAX_Y / 2);
		width = SCREEN_MAX_X; height = SCREEN_MAX_Y;
		Anchor = Point(0, 0);
	}
	// ������תװ�װ湹��ROI
	Roi(ArmorRect& rect)
	{
		center = rect.center;
		width = scale * (int)rect.long_edge;
		height = scale * (int)rect.long_edge;
		getAnchor();

		//��ֹԽ��
		if (Anchor.x < 0)
			Anchor.x = 0;
		if (Anchor.y < 0)
			Anchor.y = 0;
		if (Anchor.x + width > SCREEN_MAX_X) {
			width = SCREEN_MAX_X - Anchor.x;
		}
		if (Anchor.y + height > SCREEN_MAX_Y) {
			height = SCREEN_MAX_Y - Anchor.y;
		}
	}

	Roi(std::vector<Point> &contour)
	{
		Rect rect = boundingRect(contour);
		center = Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
		width = scale * (int)rect.width;
		height = scale * (int)rect.height;
		getAnchor(); // ���ڸ����������ţ�������һ���Ǳ�Ҫ��

		//��ֹԽ��
		if (Anchor.x < 0)
			Anchor.x = 0;
		if (Anchor.y < 0)
			Anchor.y = 0;
		if (Anchor.x + width > SCREEN_MAX_X) {
			width = SCREEN_MAX_X - Anchor.x;
		}
		if (Anchor.y + height > SCREEN_MAX_Y) {
			height = SCREEN_MAX_Y - Anchor.y;
		}
	}
	void drawRoi(Mat& frame)// ��ԭͼ����roi
	{
		rectangle(frame, Rect(Anchor.x, Anchor.y, width, height), Scalar(0, 255, 255), 2);
	}
}Roi;

// ������Ҫ�Ƕ������������������װ�װ�����������
// һ���ǵ�ǰ��װ�װ壬һ����ǰһ��װ�װ�
class Energy
{
private:
	int THRESH_BR = 56;
	int THRESH_GRAY = 79;
	int MIN_AREA = 200;						// ���п�ʶ�������С�������
	int IS_RED = 0;							// �з�װ�װ���ɫΪ��ɫ��Ϊ1
	int SHUN = 0;							// �ж���˳��ʱ�룬0��ⲻ����1Ϊ˳��2Ϊ��
	bool BIG_OR_SMALL = 0;					// С����1Ϊ���
	double NEWSPEED;						// װ�װ�����ٶ�
	kalman_filter kf;						// ���忨�����˲���
	double accelerate;						// ��ǰ���ٶ�(��ǰ����֡���ٶȼ���õ�)
	double former_speed;					// ǰһ֡���ٶ�
	double second_to_frame = 0.03;			// һ֡������
	double bias = 0;						// �Ŷ��Ĵ�С
	int DILATE_EOFF = 7;
	Roi roi;
	double radius = 3;
	Point2f fan_center;						// ����Ҷ������
    double maxSpeed = 2;
    double minSpeed = 1.2;
    double minSpeedCount = 0;
    double maxSpeedCount = 0;
    int pass_frames = 20;
    int pass_index = 0;
    std::vector<double> initBigSpeed;
    int frame_number = 0;
	// �ϴε�����ʱ���ֻ��ɵ�֡�����⣬��������ROI����Բ��
	// ��ʵ�ּ��ٶȴ���Ŷ�Ԥ��
public:
    bool isBig = 0;
	// ��Ҫ���ظ���ص�ֵ
	float pre_x = 0;						// Ԥ������x����
	float pre_y = 0;						// Ԥ������y����
	int is_find = 0;						// �Ƿ��ҵ�װ�װ�
	// ƫ��ʱ��
	float getTime;
	int time2fire = 0;

	Energy() 
	{
		kf.init_kalman_filter(1);				// �������˲��ٶȳ�ʼ��
	}
	// ��ҪһЩ�������캯��
	// ͨ��ÿһ֡ͼ��õ�һ��mask��ͼ��ʶ����Ҫ����
	void videoProcess(Mat& frame, Mat& mask, ArmorRect& present, ArmorRect& former)
	{
		getMask(frame, mask);					// �õ���Ĥ
		/* ------��mask���ҳ�����ȷ��present------- */
		getCoutour(mask, frame, present);		// ��װ�װ������ // �õ��µ�present
		getR(mask, present, former);					// �õ�����R	// Բ����Ҫװ�װ���ȷ�� // ȷ��װ�װ����������
		// ֻ�����ǰ�����������ſ���ȷ��present
		present.setToCenter();
		//if (is_find)
		//	roi.drawRoi(frame);
		if (is_find)
		{
			judgeDirect(present, former, frame);	// �ж�˳�� 
			//newSpeedMeasure(present, former);		// �ݲ�ʹ�ã��Ż���
			getSpeed(present, former, frame);		// ������ٶ�
			former = present;						// ����װ�װ���Ϣ
			drawCircle(frame, present);
			//std::cout << sqrt((fan_center.x - center_R.center.x) * (fan_center.x - center_R.center.x) +
			//	(fan_center.y - center_R.center.y) * (fan_center.y - center_R.center.y)) << std::endl;
		}
		//predict(present, frame);				// ͨ�����ٶȽ���Ԥ��
	}
	void reset()
	{
		//std::cout << pre_y << std::endl;//debug��
		pre_x = 0;
		pre_y = 0;
	}
	// �ⲿ�ӿ�:������ֵ
	void setThresh(int br, int gray) {
		THRESH_BR = br;
		THRESH_GRAY = gray;
	}
	// �ⲿ�ӿ�:�Ƿ�Ϊ��ɫ
	void isRed(bool b)
	{
		IS_RED = b;
	}
	
	void getMask(Mat& frame, Mat& mask);
	// ������ǰ֡��װ�װ��֮ǰ֡��װ�װ������ٶȻ����ǽǶȣ���Ҫ�õ����
	void getSpeed(ArmorRect& present, ArmorRect& former, Mat& frame);
	// Ԥ����Ҫ�����λ�ã���Ҫ����pitch��yaw
	void predict(ArmorRect& present, Mat& frame);

	// �ж���˳ʱ�뻹����ʱ��
	void judgeDirect(ArmorRect& present, ArmorRect& former, Mat& frame);
	// ͨ���������ҵķ����ҵ�װ�װ��λ��
	void getCoutour(Mat& mask, Mat& frame, ArmorRect& armor_center);
	// ��������Ե�����ʶ��R����ʱ��һϵ���ж�
	bool isValidCenterRContour(const std::vector<Point>& center_R_contour);
	// �õ�R���ĵĺ������õ�center_R������
	void getR(Mat& mask, ArmorRect& present, ArmorRect &former);
	bool judgeRPosition(RotatedRect& R, ArmorRect& present, ArmorRect &former);
	Point2f getCoutourCenter(const std::vector<Point>& coutour)
	{
		int n = coutour.size();
		double x_sum = 0 , y_sum = 0;
		if (n <= 0)
			return Point2f(0, 0);
		for (auto& c : coutour)
		{
			x_sum += c.x; y_sum += c.y;
		}
		return Point2f(x_sum / n, y_sum / n);// �Ƿ�������ʱ��������bug�����İ���ʱ�������еģ���������bug
	}

	// �����ݷ�װ�������͸����
	void send(double &x, double &y)
	{
		if (is_find)
		{
			x = pre_x;
			y = pre_y;
		}
		else
		{
			x = 0;
			y = 0;
		}
	}
	bool isValidCenterFansContour(const std::vector<Point>& countour);
	bool isValidCenterArmorContour(const std::vector<Point>& contour);
	void activeBig()
	{
		BIG_OR_SMALL = 1;
	}
	void activeSmall()
	{
		BIG_OR_SMALL = 0;
	}
	void newSpeedMeasure(ArmorRect& present, ArmorRect& former)
	{
		// ����㷨��Ҫ����뾶���뾶Ҳ����Ҫ
		double h = (double)(present.center.y - center_R.center.y);
		double w = (double)(present.center.x - center_R.center.x);
		//double r = sqrt(h * h + w * w);
		double delta_h = (double)(present.center.y - former.center.y);
		double delta_w = (double)(present.center.x - former.center.x);
		double radio1 = delta_h / w / PI * 180;
		double radio2 =  - delta_w / h / PI * 180;
		double eps = 0.1;
		int iteratorTime = 10;
		if (radio1 < 10 && radio2 < 10) // ���10emmm,��Ҫ��Ϊ�������쳣ֵ��������Ҫ�Ķ�
		{// ���ý�����������������
			if (h > 2 * w)
			{
				int cnt = 0;
				while ((fabs(radio1 - radio2) < eps) && cnt < iteratorTime)
				{
					delta_w = - h * radio2 * PI / 180;
					w = delta_w + (double)(former.center.x - center_R.center.x);
					radio1 = delta_h / w / PI * 180;
					++cnt;
				}
				NEWSPEED = (radio1 + radio2) / 2;

			}
			else if (w > 2 * h)
			{
				int cnt = 0;
				while ((fabs(radio1 - radio2) < eps) && cnt < iteratorTime)
				{
					delta_h = w * radio1 * PI / 180;
					h = delta_h + (double)(former.center.y - center_R.center.y);
					radio2 = - delta_w / h / PI * 180;
					++cnt;
				}
				NEWSPEED = (radio1 + radio2) / 2;

			}
				//NEWSPEED = radio1;
			else
				//NEWSPEED = (delta_w / h + delta_h / w) / 2 / PI * 180;
				NEWSPEED = (radio1 + radio2) / 2;

			std::cout << "newSpeed: " << NEWSPEED << std::endl;
		}
	}
	void bigPredict(ArmorRect& present, ArmorRect& former, Mat& frame)
	{
		if (!is_find) return;
		int frameNum = 10;// 3֡�ӳ�
		// ��ת�ĽǶȣ���������������ת
		RMvector vec = present.to_center;
		//double theta = frameNum * 3 * PI / 180;// present.speed;
		//std::cout << "speed: " << present.speed << std::endl;
		double res = cummulativeAngel(frameNum, former.speed, present.speed);
		double theta = res;
		//double theta = (frameNum * 1.305 * second_to_frame);
		//double theta = (cummulativeAngel(frameNum, former.speed, present.speed) + (frameNum * 1.305 / second_to_frame)) * PI / 180;
		//std::cout << "res: " << res << std::endl;
		//std::cout << "theta: " << theta << std::endl;
		double newx, newy;
		if (SHUN == 1)
		{
			theta = -theta;
			newx = center_R.center.x - (vec.x * cos(theta) + vec.y * sin(theta));
			newy = center_R.center.y - (-vec.x * sin(theta) + vec.y * cos(theta));
			circle(frame, Point(newx, newy), 2, Scalar(0, 255, 0), 2);
			pre_x = newx; pre_y = newy;
		}
		else if (SHUN == 2)
		{
			newx = center_R.center.x - (vec.x * cos(theta) + vec.y * sin(theta));
			newy = center_R.center.y - (-vec.x * sin(theta) + vec.y * cos(theta));
			circle(frame, Point(newx, newy), 2, Scalar(0, 255, 0), 2);
			pre_x = newx; pre_y = newy;
		}
	}
    void bigPredict2(ArmorRect& present, ArmorRect& former, Mat& frame)
    {
        if (!is_find) return;
        //int frameNum = 20;// 3???
        // ??????????????
        RMvector vec = present.to_center;
        //double theta = frameNum * 3 * PI / 180;// present.speed;
        //std::cout << "speed: " << present.speed << std::endl;
        //double second = frameNum * second_to_frame;
        double res = bigIntAngel(present, former, getTime);
        double theta = res * PI / 180;
        //std::cout << "res:: " << res << std::endl;
        double newx, newy;
        if (SHUN == 1)
        {
            theta = -theta;
            newx = center_R.center.x - (vec.x * cos(theta) + vec.y * sin(theta));
            newy = center_R.center.y - (-vec.x * sin(theta) + vec.y * cos(theta));
            circle(frame, Point(newx, newy), 2, Scalar(0, 255, 0), 2);
            pre_x = newx; pre_y = newy;
        }
        else if (SHUN == 2)
        {
            newx = center_R.center.x - (vec.x * cos(theta) + vec.y * sin(theta));
            newy = center_R.center.y - (-vec.x * sin(theta) + vec.y * cos(theta));
            circle(frame, Point(newx, newy), 2, Scalar(0, 255, 0), 2);
            pre_x = newx; pre_y = newy;
        }
    }


	void updateSpeed(ArmorRect& present, ArmorRect& former, Mat& frame)
	{
		// y = 0.785*sin(1.884*t)+1.305
		double Speed = present.speed;// ���û���ҵ�װ�װ壬������ٶȽ���֮���װ�װ���ٶ�(�Ѹ��µ��ٶ�)
		if (Speed > 0.1 && is_find)//��Ϊ0.1��ĺ���, Ҫ���ҵ���װ�װ��ʱ������ٶȸ���
		{
			Speed = kf.correct_value(Speed);
			//energy.setSpeed(Speed);
            present.setSpeed(Speed); // �����ٶ�
            predict(present, frame);				// С��Ԥ��
			//bigPredict(present, former, frame); // ���Ԥ�� --- ���Ʒ���
            if(isBig)
                bigPredict2(present, former, frame); // ���Ԥ�� --- ��������
                //bigPredict3(present, former, frame);	// ���Ԥ�� --- �˼�����
            else time2fire = 1;
			putText(frame, "UPDATE: " + std::to_string(Speed), Point(100, 100), CV_FONT_NORMAL, 1, Scalar(0, 255, 0), 2);
			//std::cout << Speed << " ";
		}
	}
	void bigPredict3(ArmorRect& present, ArmorRect& former, Mat& frame)
	{
		// �жϲ��岨��
//        if(present.speed > maxSpeed)
//        {
//            maxSpeed = present.speed;
//        }
//        if(present.speed < minSpeed && present.speed > 0.5)
//        {
//            minSpeed = present.speed;
//        }

        if (fabs(present.speed - maxSpeed) < 0.3)
		{
            maxSpeed = (present.speed + maxSpeedCount * maxSpeed) / (maxSpeedCount + 1);
            ++maxSpeedCount;
			time2fire = 1;
		}
        else if (fabs(present.speed - minSpeed) < 0.3)
		{
            minSpeed = (present.speed + minSpeedCount * minSpeed) / (minSpeedCount + 1);
            ++minSpeedCount;
			time2fire = 1;
		}
		else
			time2fire = 0;
		putText(frame, "fire: " + std::to_string(time2fire), Point(100, 130), CV_FONT_NORMAL, 1, Scalar(0, 255, 0), 2);

	}

	void calZL(ArmorRect& present)
	{
		// ����֡��
	}

	// ����(�����˫�߳� + Ӳ����)
	void getAccelerate(ArmorRect& present, ArmorRect& former)
	{
		double acc = (present.speed - former.speed);// ������ٶ�
		double omega = 1.884 * second_to_frame;// ת����λ
		bias = acc / (omega * omega);//�����Ŷ����Ŷ�Ӧ���ǵ�ǰ֡ת���ĽǶȣ�Ҳ������Ϊ��һ֡�ĽǶ�

	}
	double cummulativeAngel(int num, double f_speed, double speed)// ����num��
	{
		// ���ã��㷨����ȷ
		double sum_angel = 0;
		double omega = 1.884;
		double b;
		double pre_speed = speed;
		double angel;
		double acc;
		for (int i = 0; i < num; i++)
		{	
			//angel = (pre_speed + f_speed)/2; // ����λ������
			acc = (pre_speed - f_speed) * second_to_frame; // ʱ��Ϊ1֡
			b = acc / (omega * omega);
			//std::cout << "b: " << b << std::endl;
			// ������һ֡
			f_speed = pre_speed;
			sum_angel += b;
			//b = sum_angel + b;
			pre_speed += b * omega * omega / second_to_frame;

		}
		return sum_angel;
	}

    double bigIntAngel(ArmorRect& present, ArmorRect& former, double second)
    {
        /*------------------------------------?????-----------------------------------------*/
        // // ???????
        //double acc = (present.speed - former.speed) * PI / 180 ;// ?????
        ////std::cout << "speed1: " << present.speed << " formerSpeed: " << former.speed << std::endl;
        ////double omega = 1.884 ;// ????
        ////double b = acc / (omega * omega);//??????????????????????????????
        //double t_0 = acos(-acc  / 0.804) / 0.0384;
        //double b = -0.804 / 0.0384 * cos(0.0384 * t_0);
        //double t = t_0 + second;
        //double b_pre = -0.804 / 0.0384 * cos(0.0384 * t);
        //return b_pre - b + second * 1.704;
        /*------------------------------------?????-----------------------------------------*/
        //double t_0 = asin(present.speed  * PI / 180 / 0.785) / 0.0384;
        ////double t_0 = acos(-b * 1.884 / 0.785) / 1.884;
        //double b = -0.785 / 0.0384 * cos(0.0384 * t_0);
        //double t = t_0 + second;
        //double b_pre = -0.785 / 0.0384 * cos(0.0384 * t);
        //return b_pre - b + 1.704 * second;
        /*------------------------------------?????-----------------------------------------*/
        double h = 1;
        double precise = 1;
        second++; // ?????1?????????????
        int divi = (int)(precise * 2 * PI / 0.0384); // ???????????h//  ?????????// ?????
        double eps = 0.0001; // ????
        std::vector<double> values;
        for (int i = 0; i < divi; i++)
        {
            double temp = 0.802 * sin(2 * PI * i / divi) + 1.704;
            values.push_back(temp);
        }
        std::vector<double> minValue;
        std::vector<int> counting;
        int i = 0;
        for (auto& v : values)
        {
            double tem = fabs(present.speed - v); // ?????????
            minValue.push_back(tem);
            //err.push_back(tem);
            counting.push_back(i++);
        }
        sort(counting.begin(), counting.end(), [&minValue](int i1, int i2) {return minValue[i1] < minValue[i2]; });
        if (!pass_index)
        {
            pass_index++;
            return 0; // ????????????
        }
        if (pass_index < pass_frames)
        {
            pass_index++;
            initBigSpeed.push_back(counting[0]); // ?????
            return 0;
        }
        if (pass_index == pass_frames)
        {
            sort(initBigSpeed.begin(), initBigSpeed.end());
            double med = initBigSpeed[5];
            for (auto& v : initBigSpeed)
                if (fabs(v - med) > 10)
                    v = 0;
            double max = 0;
            for (auto& v : initBigSpeed)
                if (v > max)
                    max = v;
            present.minErrorIndex = max;
            pass_index++;
            return 0;
        }

        ////std::cout << "present.minErrorIndex: " << present.minErrorIndex << std::endl;
        // predict seconds frames;
        present.minErrorIndex = (former.minErrorIndex + 1) % divi;
        std::vector<double> pre;
        for (int i = 0; i < second + 1; i++)
        {
            pre.push_back(values[(present.minErrorIndex + i) % divi]);
        }
        // calculate the integrate of the points; using the NEWTON method. Simpose method is also viable
        double sum = 0;
        for (int i = 0; i < second; i++) // ?????????????????????????
        {
            sum += h * (pre[i] + pre[i + 1])/2;
        }
        //for (int i = 0; i < second - 1; i++)// ????? // ?????????????
        //{
        //	sum += h * (pre[i] + 4 * pre[i + 1] + pre[i + 2])/6;
        //}
        std::cout << "sum: " << sum << std::endl;
        std::cout << "index: " << present.minErrorIndex << std::endl;
        return sum;
        /*------------------------------------?????-----------------------------------------*/
        // ?????
        // ????????????????????????
        // ??????????????????????????????????
        // ?????????+arima?? ???????????????????????????
    }


	void drawCircle(Mat& frame, ArmorRect& present)
	{
		circle(frame, center_R.center, present.to_center.getNorm(), Scalar(240, 240, 100), 2);
	}
};
