# include "Energy.h"

RotatedRect center_R = RotatedRect(Point2f(0, 0), Point2f(0, 0), Point2f(0, 0));
int SCREEN_MAX_X = 1280;
int SCREEN_MAX_Y = 720;
const double PI = 3.1415926535;

/******************************Energy*****************************/
// ����ʶ���Ͽ��Լ���������õ�������
void Energy::getMask(Mat& frame, Mat& mask)
{
	std::vector<Mat> BGR;
	Mat BR, gray, gray_mask;
	split(frame, BGR);

	//for (int i = 0; i < 3; i++)
	//{
	//	BGR[i].convertTo(BGR[i], CV_64F, 1.0 / 255, 0);
	//	pow(BGR[i], 10, BGR[i]);
	//	BGR[i].convertTo(BGR[i], CV_8U, 255.0, 0);
	//}
	//Mat temp;
	//merge(BGR, temp);
	//cvtColor(temp, gray, COLOR_BGR2GRAY);//���ɻҶ�ͼ
	cvtColor(frame, gray, COLOR_BGR2GRAY);//���ɻҶ�ͼ

	//gray.convertTo(gray, CV_64F, 1.0 / 255, 0);
	//pow(gray, 0.4, gray);
	//gray.convertTo(gray, CV_8U, 255.0, 0);
	//imshow("gray", gray);
	// if DEBUG
	//threshold(gray, gray_mask, 0, 255, THRESH_OTSU);
	threshold(gray, gray_mask, THRESH_GRAY, 255, THRESH_BINARY);

	if (IS_RED)
		BR = BGR[2] - BGR[0];
	else
		BR = BGR[0] - BGR[2];
	threshold(BR, mask, THRESH_BR, 255, THRESH_BINARY); // ͨ������ͨ�����ʶ��
	// if DEBUG
	// threshold(BR, mask, 0, 255, THRESH_OTSU); 
	mask &= gray_mask;//ʹ�������

	morphologyEx(mask, mask, MORPH_DILATE, getStructuringElement(MORPH_RECT, Size(DILATE_EOFF, DILATE_EOFF)));// ���������3����5
	//morphologyEx(mask, mask, MORPH_DILATE, getStructuringElement(MORPH_RECT, Size(3, 3)));// ���������3����5
    morphologyEx(mask, mask, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size(3, 3)));
}

// ������ٶ��������㷨�ĺ��ģ���������Ҫ������������Ҫ����
void Energy::getSpeed(ArmorRect& present, ArmorRect& former, Mat& frame)
{
	if (!is_find) return;
	double cos_theta = present.to_center.dot(former.to_center) / (present.to_center.getNorm() * former.to_center.getNorm());
    // std::cout<< abs(acos(cos_theta) * 180 / PI) << std::endl;
    std:: cout << present.to_center.getNorm()<<std::endl;
    std:: cout << former.to_center.getNorm()<<std::endl;

	if (acos(cos_theta) == acos(cos_theta) && abs(acos(cos_theta) * 180 / PI) < 10)
	{
		double thetas[4];
		double mean = 0;
		int count_num = 0;

		for (int i = 0; i < 4; i++)
		{
			RMvector vec1 = present.vertex2center[i];
			RMvector vec2 = former.vertex2center[i];
			thetas[i] = acos(vec1.dot(vec2) / (vec1.getNorm() * vec2.getNorm()));
		}

		for (auto& t : thetas)
		{
			if (t / PI * 180 < 10 && t / PI * 180 > 0.5)
			{
				mean += t * 180 / PI;
				count_num += 1;
				//std::cout << t * 180 / PI << " ";
			}
		}
		//std::cout << mean / count_num;
		//std::cout << std::endl;
        //std::cout << acos(cos_theta) * 180 / PI << std::endl;

		present.setSpeed((acos(cos_theta) * 180 / PI + mean) / (count_num + 1));
		putText(frame, "Angle/frame" + std::to_string((acos(cos_theta) * 180 / PI + mean)/ (count_num + 1)), Point(100, 30), CV_FONT_NORMAL, 1, Scalar(0, 255, 0), 2);
		//putText(frame, "Angle" + std::to_string((present.to_center.angle) / PI * 180), Point(100, 30), CV_FONT_NORMAL, 1, Scalar(0, 255, 0), 2);
	}
}

// �������Ԥ���ǰ���ǿ���ʶ�����˳ʱ�뻹����ʱ�룬���﹦����ϣ���Ҫ���ģ���Ӧ�����
void Energy::predict(ArmorRect& present, Mat& frame)
{
	if (!is_find) return;
	float frameNum = 10;// 3֡�ӳ�
	// ��ת�ĽǶȣ���������������ת
	RMvector vec = present.to_center;
	//double theta = frameNum * 3 * PI / 180;// present.speed;
	//std::cout << "speed: " << present.speed << std::endl;

	double theta = getTime * present.speed * PI / 180;
	//double theta = frameNum * present.speed * PI / 180;
	double newx, newy;
	if (SHUN == 1)
	{
		theta = -theta;
		newx = center_R.center.x - (vec.x * cos(theta) + vec.y * sin(theta));
		newy = center_R.center.y - (-vec.x * sin(theta) + vec.y * cos(theta));
		circle(frame, Point(newx, newy), 2, Scalar(0, 255, 0), 2);
		pre_x = newx; pre_y = newy;
	}
	else if(SHUN == 2)
	{
		newx = center_R.center.x - (vec.x * cos(theta) + vec.y * sin(theta));
		newy = center_R.center.y - (-vec.x * sin(theta) + vec.y * cos(theta));
		circle(frame, Point(newx, newy), 2, Scalar(0, 255, 0), 2);
		pre_x = newx; pre_y = newy;
	}
}

// �޸����
void Energy::judgeDirect(ArmorRect& present, ArmorRect& former, Mat& frame)
{	
	if (!is_find) return;
	float res = former.to_center.cross(present.to_center);
	float Cos = former.to_center.dot(present.to_center) / (former.to_center.getNorm() * present.to_center.getNorm());
	if (res && fabs(Cos) > 0.9)
	{
		if (res > 0)
		{
			putText(frame, "shun", Point(100, 60), CV_FONT_NORMAL, 1, Scalar(0, 255, 0), 2);
			SHUN = 1;
		}
		else
		{
			putText(frame, "ni", Point(100, 60), CV_FONT_NORMAL, 1, Scalar(0, 255, 0), 2);
			SHUN = 2;
		}
	}
	else SHUN = 0; // �������һ������Ϊ��ҶͻȻ��������ط����ˣ����Դ�ʱ���ü����ٶȣ����쳣ֵ
	// �ڵ�����ʱ����Ҷ�Ǿ�ֹ�ģ����ʱ����Ϊ0
}

void Energy::getCoutour(Mat& mask, Mat& frame, ArmorRect& armor_center)
{
	std::vector<std::vector<Point> > armorContours;
	std::vector<Vec4i> armorHierarchy;
	findContours(mask, armorContours, armorHierarchy, RETR_TREE, CHAIN_APPROX_NONE);
	int find_flag = 0; // �����ж��Ƿ��ҵ�
	// Hierarchy[i][0-4]: next,previous,first_child, parent

	// Ѱ��Ŀ��
	int contour_size = armorContours.size();
	int* count = new int[contour_size];
	for (int k = 0; k < contour_size; k++)
		count[k] = 0;// ����

	for (int i = 0; i < contour_size; i++)
	{
		if (armorHierarchy[i][3] != -1)// �и�����
		{
			if (contourArea(armorContours[i]) > MIN_AREA) {// ��Ҫ�� ʹ��count��¼����������ȷ��
				count[armorHierarchy[i][3]]++;
			}
		}
	}
	for (int i = 0; i < contour_size; i++)
	{
		//��¼����һ��������������������������countû�������ļ�¼����ֹ��������
		if (count[i] == 1 && contourArea(armorContours[i]) > MIN_AREA)
		{
			// ��ǰ���Һ������ң�Ѱ�����������������һ������������
			int now = armorHierarchy[i][2];// now���������ı��
			double max_area = (double)contourArea(armorContours[now]);
			int max_armor = now;
			while (armorHierarchy[now][0] != -1)
			{
				now = armorHierarchy[now][0];
				if ((double)contourArea(armorContours[now]) > max_area)
				{
					max_area = (double)contourArea(armorContours[now]);
					max_armor = now;
				}
			}
			now = armorHierarchy[i][2];
			while (armorHierarchy[now][1] != -1)
			{
				now = armorHierarchy[now][1];
				if ((double)contourArea(armorContours[now]) > max_area)
				{
					max_area = (double)contourArea(armorContours[now]);
					max_armor = now;
				}
			}

			// ����ȷ�ֹ��ʶ�� // ��ʵ�ϣ����ʶ���Ѿ��ܺ��ˣ����Ƿ���Ҫ���������أ�
			RotatedRect rotate_rect = minAreaRect(armorContours[max_armor]);
			//double long_edge = max(rotate_rect.size.width, rotate_rect.size.height);
			//double short_edge = min(rotate_rect.size.width, rotate_rect.size.height);
			//double wh_rio = long_edge / short_edge;
			//if (wh_rio > 1.2 && wh_rio < 2.5)
			if(isValidCenterArmorContour(armorContours[max_armor]))
			{
				Point2f vertices[4];
				rotate_rect.points(vertices);
				for (int j = 0; j < 4; j++)// ������ת����Debug
					//line(mask, vertices[j], vertices[(j + 1) % 4], 127, 2);
					line(frame, vertices[j], vertices[(j + 1) % 4], Scalar(0, 255, 0), 3);
				// һ����˵�������ȷʶ�𵽣���ôֻ�᷵��һ��װ�װ�
				find_flag = 1;
				armor_center.setParam(rotate_rect);
				circle(frame, armor_center.center, 1, Scalar(0, 0, 255), 2);
				// ������ҶROI
				roi = Roi(armorContours[i]);
                pre_x = armor_center.center.x;
                pre_y = armor_center.center.y;
			}
		}

	}
	is_find = find_flag;
	delete[] count;
	//// ��ֵͼ�������������(����Debug)
	//for (int i = 0; i < armorContours.size(); i++)
	//{
	//	RotatedRect rotate_rect = minAreaRect(armorContours[i]);
	//	Point2f vertices[4];
	//	rotate_rect.points(vertices);
	//	for (int j = 0; j < 4; j++)// ������ת����Debug
	//		line(mask, vertices[j], vertices[(j + 1) % 4], 127, 2);
	//}
}
// ��������Ե���
bool Energy::isValidCenterRContour(const std::vector<Point>& center_R_contour) {
	//ѡ�������С������
	double cur_contour_area = contourArea(center_R_contour);
    if (cur_contour_area > 1500 || cur_contour_area < 100)
		return false;

	//���α߳�������
	RotatedRect cur_rect = minAreaRect(center_R_contour);
	Size2f cur_size = cur_rect.size;
	float length = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;//�����εĳ�������Ϊ��
	float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;//�����εĶ̱�����Ϊ��
    if (length < 10 || width < 10 || length >  100 || width > 100)
		return false;

	//����Ȳ�����
	float length_width_ratio = length / width;//������γ����
    if (length_width_ratio > 3 || length_width_ratio < 1)
		return false;

	// �����Ծ��ε����ռ���ʲ�����
    if (cur_contour_area / cur_size.area() < 0.6)
		return false;
	return true;
}

bool Energy::isValidCenterFansContour(const std::vector<Point>& contour)
{
	//ѡ�������С������
	double cur_contour_area = contourArea(contour);
	if (cur_contour_area > 25000 || cur_contour_area < 3000)
		return false;

	//���α߳�������
	RotatedRect cur_rect = minAreaRect(contour);
	Size2f cur_size = cur_rect.size;
	float length = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;//�����εĳ�������Ϊ��
	float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;//�����εĶ̱�����Ϊ��
	if (length < 100 || width < 100 || length >  500 || width > 500)
		return false;

	//����Ȳ�����
	float length_width_ratio = length / width;//������γ����
	if (length_width_ratio > 10 || length_width_ratio < 1.5)
		return false;

	// �����Ծ��ε����ռ���ʲ�����
	if (cur_contour_area / cur_size.area() < 0.2)
		return false;
	
	return true;
}

bool Energy::isValidCenterArmorContour(const std::vector<Point>& contour)
{
	//ѡ�������С������
	double cur_contour_area = contourArea(contour);
	if (cur_contour_area > 5000 || cur_contour_area < 100)//1548
		return false;

	//���α߳�������
	RotatedRect cur_rect = minAreaRect(contour);
	Size2f cur_size = cur_rect.size;
	float length = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;//�����εĳ�������Ϊ��
	float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;//�����εĶ̱�����Ϊ��
	if (length < 10 || width < 10 || length >  100 || width > 80)// 60, 30
		return false;

	//����Ȳ�����
	float length_width_ratio = length / width;//������γ����
	if (length_width_ratio > 2.5 || length_width_ratio < 1.2)
		return false;

	// �����Ծ��ε����ռ���ʲ�����
	if (cur_contour_area / cur_size.area() < 0.7)
		return false;

	return true;
}

bool Energy::judgeRPosition(RotatedRect &R, ArmorRect &present)
{
	// R.center �� present����֮��ľ���
	RMvector vec(R.center, present.center);
	RMvector armor_nvector(present.angle);
	RMvector armor_tvector(present.tangent_angle);
	//std::cout << "-------------------------------------------------------------"<< std::endl;
	//std::cout << "x: " << vec.x / vec.getNorm() << " y: " << vec.y / vec.getNorm() << std::endl;
	//std::cout << "xn: " << armor_nvector.x << " yn: " << armor_nvector.y << std::endl;
	//std::cout << "xt: " << armor_tvector.x << " yt: " << armor_tvector.y << std::endl;

	//std::cout << "foeofwnfo: " << fabs(vec.dot(armor_tvector) / vec.getNorm()) << std::endl;
	//std::cout << "ofwnfo: " << fabs(vec.dot(armor_nvector) / vec.getNorm()) << std::endl;
	//std::cout << "foo: " << vec.getNorm() / present.long_edge << std::endl;

	// �ǶȲ���������
    if (fabs(vec.dot(armor_tvector) / vec.getNorm()) > 0.15 || fabs(vec.dot(armor_nvector) / vec.getNorm()) < 0.9)
	{
		return false;
	}
	
    if (vec.getNorm() / present.long_edge > 10 || vec.getNorm() / present.long_edge < 1) // 3.2����
	{
		return false;
	}
	return true;
}

void Energy::getR(Mat& mask, ArmorRect& present)
{
	RotatedRect R;
	std::vector<std::vector<Point> > R_contours;
	findContours(mask, R_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	for (auto& R_contour : R_contours) {
		if (!isValidCenterRContour(R_contour)) { // �ж�����
			continue;
		}
		R = minAreaRect(R_contour);
		if (judgeRPosition(R, present))
		{
			Point2f vertices[4];
			center_R.points(vertices);
			for (int j = 0; j < 4; j++)
				line(mask, vertices[j], vertices[(j + 1) % 4], 127, 2);
			center_R = R;
			break;
		}
	}
}
