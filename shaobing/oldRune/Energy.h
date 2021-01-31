#pragma once

#include <opencv2/opencv.hpp>
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
		angle = a;
	}
	double getNorm()// ���������ĳ���
	{
       // double res = sqrt(x * x + y * y);
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
        //speed = 0;
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
        //speed = 0;
		hitted = 0;

		rect.points(vertices);
	}
	void setSpeed(double w)
	{
		speed = w;
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
    int SHUN = 1;							// �ж���˳��ʱ�룬0��ⲻ����1Ϊ˳��2Ϊ��
	bool BIG_OR_SMALL = 0;					// С����1Ϊ���
	double NEWSPEED;						// װ�װ�����ٶ�
	kalman_filter kf;						// ���忨�����˲���
	double accelerate;						// ��ǰ���ٶ�(��ǰ����֡���ٶȼ���õ�)
	double former_speed;					// ǰһ֡���ٶ�
	double second_to_frame = 0.03;			// һ֡������
	double bias = 0;						// �Ŷ��Ĵ�С
    int DILATE_EOFF = 7;
	Roi roi;

	// �ϴε�����ʱ���ֻ��ɵ�֡�����⣬��������ROI����Բ��
	// ��ʵ�ּ��ٶȴ���Ŷ�Ԥ��
public:
	// ��Ҫ���ظ���ص�ֵ
	float pre_x = 0;						// Ԥ������x����
	float pre_y = 0;						// Ԥ������y����
	int is_find = 0;						// �Ƿ��ҵ�װ�װ�
	// ƫ��ʱ��
	float getTime;
	Energy() 
	{
        kf.init_kalman_filter(3);				// �������˲��ٶȳ�ʼ��
	}
	// ��ҪһЩ�������캯��
	// ͨ��ÿһ֡ͼ��õ�һ��mask��ͼ��ʶ����Ҫ����
	void videoProcess(Mat& frame, Mat& mask, ArmorRect& present, ArmorRect& former)
	{
		getMask(frame, mask);					// �õ���Ĥ
		/* ------��mask���ҳ�����ȷ��present------- */
		getCoutour(mask, frame, present);		// ��װ�װ������ // �õ��µ�present
		getR(mask, present);					// �õ�����R	// Բ����Ҫװ�װ���ȷ�� // ȷ��װ�װ����������
		// ֻ�����ǰ�����������ſ���ȷ��present
		present.setToCenter();
		//if (is_find)
		//	roi.drawRoi(frame);
		if (is_find)
		{
            //judgeDirect(present, former, frame);	// �ж�˳��
            //newSpeedMeasure(present, former);
			getSpeed(present, former, frame);		// ������ٶ�
            //std::cout<<present.speed<<std::endl;
			former = present;						// ����װ�װ���Ϣ
            former.to_center = present.to_center;
            //predict(present, frame);				// ͨ�����ٶȽ���Ԥ��

		}
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
	void getR(Mat& mask, ArmorRect& present);
	bool judgeRPosition(RotatedRect& R, ArmorRect& present);


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
		double h = fabs(present.center.y - center_R.center.y);
		double w = fabs(present.center.x - center_R.center.x);
		//double r = sqrt(h * h + w * w);
		double delta_h = fabs(present.center.y - former.center.y);
		double delta_w = fabs(present.center.x - former.center.x);
		double radio1 = delta_h / w / PI * 180;
		double radio2 = delta_w / h / PI * 180;
		if (radio1 < 10 && radio2 < 10)
		{
			if (h > 2 * w)
				NEWSPEED = radio2;
			else if (w > 2 * h)
				NEWSPEED = radio1;
			else
				NEWSPEED = (delta_w / h + delta_h / w) / 2 / PI * 180;
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
		std::cout << "theta: " << theta << std::endl;
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
		int frameNum = 10;// 3֡�ӳ�
		// ��ת�ĽǶȣ���������������ת
		RMvector vec = present.to_center;
		//double theta = frameNum * 3 * PI / 180;// present.speed;
		//std::cout << "speed: " << present.speed << std::endl;
		double second = frameNum * second_to_frame;
		double res = bigIntAngel(present, former, second);
		double theta = res;
		std::cout << "res:: " << res << std::endl;
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
        //std::cout<<present.speed<<std::endl;
		if (Speed > 0.1)//��Ϊ0.1��ĺ���
		{
			Speed = kf.correct_value(Speed);
			//energy.setSpeed(Speed);
			present.setSpeed(Speed); // �����ٶ�
			predict(present, frame);				// С��Ԥ��
			//bigPredict(present, former, frame); // ���Ԥ�� --- ���Ʒ���
			//bigPredict2(present, former, frame); // ���Ԥ�� --- ��������
			putText(frame, "UPDATE: " + std::to_string(Speed), Point(100, 100), CV_FONT_NORMAL, 1, Scalar(0, 255, 0), 2);

		}

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
			std::cout << "b: " << b << std::endl;
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
		/*------------------------------------��һ�׷���-----------------------------------------*/
		//double acc = (present.speed - former.speed) * PI / 180 / second_to_frame;// ������ٶ�
		////std::cout << "speed1: " << present.speed << " formerSpeed: " << former.speed << std::endl;
		////double omega = 1.884 ;// ת����λ
		////double b = acc / (omega * omega);//�����Ŷ����Ŷ�Ӧ���ǵ�ǰ֡ת���ĽǶȣ�Ҳ������Ϊ��һ֡�ĽǶ�
		//double t_0 = acos(-acc  / 0.785) / 1.884;
		//double b = -0.785 / 1.884 * cos(1.884 * t_0);
		//double t = t_0 + second;
		//double b_pre = -0.785 / 1.884 * cos(1.884 * t);
		//return b_pre - b;
		/*------------------------------------�ڶ��׷���-----------------------------------------*/
		//double t_0 = asin(present.speed  * PI / 180 / 0.785) / 1.884 / second_to_frame;
		////double t_0 = acos(-b * 1.884 / 0.785) / 1.884;
		//double b = -0.785 / 1.884 * cos(1.884 * t_0);
		//double t = t_0 + second;
		//double b_pre = -0.785 / 1.884 * cos(1.884 * t);
		//return b_pre - b;
	}
};
