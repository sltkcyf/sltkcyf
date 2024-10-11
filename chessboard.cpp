//计算与X轴正方向的夹角,范围在0-360°
double calculateAngleWithXAxis(cv::Point2f point, cv::Point2f O_pt)
{
	// 点与原点重合
	if (point == O_pt)
		return  DBL_MAX;

	// 定义x轴正方向单位向量
	cv::Point2f xAxis(1, 0);

	// 计算点与原点的向量
	cv::Point2f vector = point - O_pt;// cv::Point(1022, 723);

	// 计算点积
	double dotProduct = vector.dot(xAxis);

	// 计算模
	double magnitudeVector = cv::norm(vector);

	// 计算夹角的余弦值
	double cosTheta = dotProduct / magnitudeVector;

	// 计算夹角，转换为0到360度之间
	double angle = acos(cosTheta) * 180.0 / CV_PI;
	if (point.y > O_pt.y) {
		angle = 360 - angle;
	}

	return angle;
}

bool is_Around(std::vector<cv::Point2f>& pts, float thres, cv::Mat& show, bool save_img,cv::Point2f center)
{
	// 按照相对于中心点的角度对点进行排序
	std::sort(pts.begin(), pts.end(), std::bind(comparePointsByAngle, std::placeholders::_1, std::placeholders::_2,center));  
	//添加首点，计算两两之间距离
	pts.push_back(pts[0]);
	std::vector<double> dis_neighbour;

	//两两距离更新到x坐标
	std::vector<cv::Point2f> pts1(pts.size());
	adjacent_difference(pts.begin(), pts.end(), pts1.begin(), [](const cv::Point2f& lhs, const cv::Point2f& rhs)
	{
		return cv::Point2f(cv::norm(lhs - rhs), 0.0);
	});
	//移除第一个元素
	auto it = pts1.begin();
	pts1.erase(it);

	//两两距离差异更新到x坐标
	std::vector<cv::Point2f> pts2(pts.size());
	adjacent_difference(pts1.begin(), pts1.end(), pts2.begin(), [](const cv::Point2f& lhs, const cv::Point2f& rhs)
	{
		return cv::Point2f(lhs.x - rhs.x, 0.0);
	});
	auto iit = pts2.begin();
	pts2.erase(iit);

	//有一组之间距离差异过大，则判定NG
	//int thres_dis = 10;
	if (std::any_of(pts2.begin(), pts2.end(), [thres](const cv::Point2f& pt)
	{return abs(pt.x) > thres; }))
	{
		log_function("around pts not continous");
		if (save_img){
			imwrite("ChessBoardChart_Error.tiff", show);
		}
		return false;
	}
	return true;
}


//判断找到的点是否是连续的
bool is_Continous(const std::vector<std::pair<RotatedRect, float>>& pts, float median_rect_width,cv::Mat& show, bool save_img)
{
	for (int i = 0; i < pts.size(); i++){
		circle(show, pts.at(i).first.center + Point2f(0.0f, show.rows / 4), 20, Scalar(255, 255, 255), 2, 8);
	}
	//相邻两两做差求x坐标差值
	std::vector<std::pair<RotatedRect, float>> pt_x_gap(pts.size());
	adjacent_difference(pts.begin(), pts.end(), pt_x_gap.begin(), [](const std::pair<RotatedRect, float>& lhs, const std::pair<RotatedRect, float>& rhs)
	{
		return std::make_pair(lhs.first, cv::norm(lhs.first.center - rhs.first.center));
	});
	auto it = pt_x_gap.begin();
	pt_x_gap.erase(it);

	std::vector<std::pair<RotatedRect, float>> pt_gap_dispatiry(pt_x_gap.size());
	adjacent_difference(pt_x_gap.begin(), pt_x_gap.end(), pt_gap_dispatiry.begin(), [](const std::pair<RotatedRect, float>& lhs, const std::pair<RotatedRect, float>& rhs)
	{
		return std::make_pair(lhs.first, abs(lhs.second - rhs.second));
	});
	auto iit = pt_gap_dispatiry.begin();
	pt_gap_dispatiry.erase(iit);

	if (std::any_of(pt_gap_dispatiry.begin(), pt_gap_dispatiry.end(), [median_rect_width](const std::pair<RotatedRect, float>& elem)
	{return elem.second > median_rect_width* 1.5; }))
	{
		log_function("rect not continous");
		if (save_img){
			imwrite("Chart_Error.tiff", show);
		}
		return false;
	}
	log_function("rect is continous");
	return true;
}

//单峰三角阈值法
void TriangleUnimodal(cv::Mat &src, int& idx)
{
	cv::Mat result = cv::Mat::zeros(src.size(), CV_8UC1);
	//统计直方图
	cv::Mat hist = cv::Mat::zeros(1, 256, CV_32FC1);
	for (int i = 0; i < src.rows; ++i)
	{
		for (int j = 0; j < src.cols; ++j){
			hist.at<float>(0, src.at<uchar>(i, j))++;
		}
	}
	hist.at<float>(0, 255) = 0;
	hist.at<float>(0, 0) = 0;
	//搜索最大值位置
	float max = 0;
	int maxidx = 0;
	for (int i = 0; i < 256; ++i)
	{
		if (hist.at<float>(0, i) > max)
		{
			max = hist.at<float>(0, i);
			maxidx = i;
		}
	}
	//判断最大点在哪一侧，true为左侧，false为右侧
	bool lr = maxidx < 127;
	float maxd = 0;
	int maxdidx = 0;
	//假设在左侧
	if (lr)
	{
		float A = float(-max);
		float B = float(maxidx - 255);
		float C = float(max * 255);
		for (int i = maxidx + 1; i < 256; ++i)
		{
			float x0 = float(i);
			float y0 = hist.at<float>(0, i);
			float d = abs(A * x0 + B * y0 + C) / std::sqrt(A * A + B * B);
			if (d > maxd)
			{
				maxd = d;
				maxdidx = i;
			}
		}
	}
	// 假设在右侧
	else {
		float A = float(-max);
		float B = float(maxidx);
		float C = 0.0f;
		for (int i = 0; i < maxidx; ++i)
		{
			float x0 = float(i);
			float y0 = hist.at<float>(0, i);
			float d = abs(A * x0 + B * y0 + C) / std::sqrt(A * A + B * B);
			if (d > maxd){
				maxd = d;
				maxdidx = i;
			}
		}
	}
	//二值化
	src.setTo(255, src > maxdidx);
	src.setTo(0, src <= maxdidx);
	idx = maxdidx;
}


CHARTCHECK_API ErrorCode ChessBoardChartCheck(IN const SCharkCheckParam& inParam, OUT SCharkCheckRes* outRes)
{
	int width = inParam.img_widht;
	int height = inParam.img_height;
	Mat matGray;
	{
		Mat src(inParam.img_height, inParam.img_widht, CV_16UC1, inParam.pImg_unpack10);
		matGray = src / 4;
	}
	matGray.convertTo(matGray, CV_8UC1);
	Mat matShow_forSave = matGray.clone(); //保存图像
	//截取中间图像的一半计算
	Mat cvroiImage = matGray(Rect(0, height / 4, width, height / 2));
	//输出直方图
	if (inParam.bDebug)
	{
		cv::Mat hist;
		int histSize = 256;
		float range[] = { 0, 256 };
		const float *histRange = { range };
		bool uniform = true;
		bool accumulate = false;
		calcHist(&cvroiImage, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
		int hist_w = 500;
		int hist_h = 600;
		int bin_w = cvRound((double)hist_w / histSize);  //两个灰度级之间的距离
		Mat histImage = Mat::zeros(hist_h, hist_w, CV_8UC3); //直方图画布
		// 归一化直方图数据
		normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat()); //将数据规皈依到0和hist_h之间
		// 绘制直方图曲线
		for (int i = 1; i < histSize; i++) {
			line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(hist.at<float>(i - 1))),
				Point(bin_w*(i), hist_h - cvRound(hist.at<float>(i))), Scalar(0, 255, 0), 2, 8, 0);
		}
		imwrite("Hist.tiff", histImage);
	}
	GaussianBlur(cvroiImage, cvroiImage, Size(3, 3), 3, 7);

	//二值化选择
	if (inParam.binaryzationmethod == BinaryzationMethod::TRIANGLE)
	{
		//单峰三角二值化
		int thresh = 0;
		TriangleUnimodal(cvroiImage, thresh);
	}
	else if (inParam.binaryzationmethod == BinaryzationMethod::ADAPTIVE)
	{
		//cv::adaptiveThreshold(cvroiImage, cvroiImage, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 101, 5);//自适应阈值
		int block_size = (width / inParam.adaptive_block) / 2 * 2 + 1;
		cv::adaptiveThreshold(cvroiImage, cvroiImage, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, block_size, inParam.offset_c);
	}
	else
	{
		cv::threshold(cvroiImage, cvroiImage, inParam.binary_thres, 255, CV_THRESH_BINARY); //32
	}

	cv::Mat cvroiImage_copy = cvroiImage.clone();
	//闭运算
	Mat element = getStructuringElement(MORPH_RECT, Size(11, 11));
	morphologyEx(cvroiImage, cvroiImage, MORPH_DILATE, element, Point(-1, -1), 1, 0);
	//抓黑格轮廓
	vector<vector<Point> > Contours;
	vector<Vec4i> Hierarchy;
	findContours(cvroiImage, Contours, Hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE, Point(0, 0));
	if (inParam.bDebug)
	{
		Mat matContours = Mat::zeros(cvroiImage.size(), CV_8UC3);
		for (int i = 0; i < Contours.size(); i++)
		{
			Scalar color(255, 255, 255);
			drawContours(matContours,	//画布
				Contours,		//所有输入的轮廓
				i,				//指定要绘制轮廓的索引，如果是负数，则绘制所有的轮廓
				color,			//轮廓颜色
				3,				//画轮廓线粗，如果是负数表示填充
				8,				//轮廓线型，绘制轮廓的线的连通性
				Hierarchy,		//关于层级的可选参数，只有绘制部分轮廓时才会用到
				-1,				//maxLevel绘制轮廓的最大层数，只有hierarchy有效时才有效。
				Point(0, 0));	//偏移量
		}
		imwrite("Contours.tiff", matContours);
	}

	//记录所有grid(黑白格)
	std::vector<std::pair<RotatedRect, std::pair<double, std::vector<Point>>>> rects_all;

	const std::pair<double, double> rate_range = std::make_pair(1 - inParam.rect_wh_ratio, 1 + inParam.rect_wh_ratio);
	const std::pair<double, double> width_range = std::make_pair(20, 500);
	//1.初次筛选S: 对于宽高不符合要求的轮廓剔除掉
	//(1)符合条件的筛选出放到容器中
	std::vector<std::pair<RotatedRect, std::pair<float, std::vector<Point>>>> rects_S1;//<最小旋转矩形rotated,矩形的宽width和height的avg,轮廓点集>
	for_each(Contours.begin(), Contours.end(), [rate_range, width_range, &rects_S1, &rects_all](const vector<Point>& contour)
	{
		RotatedRect rotate_rect = minAreaRect(contour);
		double wh_rate = rotate_rect.size.width / rotate_rect.size.height;
		if (range_in(wh_rate, rate_range) && range_in(static_cast<double>(rotate_rect.size.width), width_range)){
			rects_S1.push_back(make_pair(rotate_rect, make_pair(((rotate_rect.size.width + rotate_rect.size.height) / 2), contour)));
			rects_all.push_back(make_pair(rotate_rect, make_pair(0.0, contour)));
		}
	});


	// 计算图像的中心与图像中心所在格子中心的xy差异(图像中心所在格子(黑白)中心 - 图像中心)
	// 找白格的中心位置
	//图像中心数据
	cv::Point2f center_img{ static_cast<float>(cvroiImage_copy.cols / 2.0), static_cast<float>(cvroiImage_copy.rows / 2.0) };
	{
		//图像反转
		bitwise_not(cvroiImage_copy, cvroiImage_copy);
		morphologyEx(cvroiImage_copy, cvroiImage_copy, MORPH_DILATE, element, Point(-1, -1), 1, 0);
		//抓白格子轮廓
		vector<vector<Point> > Contours1;
		vector<Vec4i> Hierarchy1;
		findContours(cvroiImage_copy, Contours1, Hierarchy1, RETR_LIST, CHAIN_APPROX_SIMPLE, Point(0, 0));
		//找旋转矩形
		for_each(Contours1.begin(), Contours1.end(), [rate_range, width_range, &rects_all](const vector<Point>& contour)
		{
			RotatedRect rotate_rect = minAreaRect(contour);
			double wh_rate = rotate_rect.size.width / rotate_rect.size.height;
			if (range_in(wh_rate, rate_range) && range_in(static_cast<double>(rotate_rect.size.width), width_range)){
				rects_all.push_back(make_pair(rotate_rect, make_pair(0.0, contour)));
			}
		});

		//计算图像中心到全部矩形框的中心距离
		for_each(rects_all.begin(), rects_all.end(), [center_img](std::pair<RotatedRect, std::pair<double, std::vector<Point>>>& rect)
		{
			rect.second.first = static_cast<double>(cv::norm(center_img - rect.first.center));
		});
		//找到最近的矩形为所在格子(包含黑白格)
		auto min_distance_rt = *std::min_element(rects_all.begin(), rects_all.end(), [](const std::pair<RotatedRect, std::pair<double, std::vector<Point>>>& lhs, const std::pair<RotatedRect, std::pair<double, std::vector<Point>>>& rhs)
		{return lhs.second.first < rhs.second.first; });

		//计算xy diff
		outRes->pt_center = std::make_pair(min_distance_rt.first.center + Point2f(0.0f, height / 4), center_img + Point2f(0.0f, height / 4));
		outRes->diff_center = std::make_pair(min_distance_rt.first.center.x - center_img.x, min_distance_rt.first.center.y - center_img.y);
		circle(matShow_forSave, outRes->pt_center.first, 6, Scalar(255, 255, 255), 2, 8);
		circle(matShow_forSave, outRes->pt_center.second, 6, Scalar(255, 255, 255), 2, 8);
		cv::drawMarker(matShow_forSave, outRes->pt_center.first, Scalar(255, 255, 255), cv::MARKER_CROSS, 15, 2);
		cv::drawMarker(matShow_forSave, outRes->pt_center.second, Scalar(255, 255, 255), cv::MARKER_STAR, 15, 2);
	}


	//2.求宽度的中位数Median
	//(1)标为k的元素放在了正确位置，对其它元素并没有排序，k左边元素都小于等于它，右边元素都大于等于它
	std::nth_element(rects_S1.begin(), rects_S1.begin() + cvCeil(rects_S1.size() / 2.0), rects_S1.end(),
		[](const std::pair<RotatedRect, std::pair<float, std::vector<Point>>>& lhs, const std::pair<RotatedRect, std::pair<float, std::vector<Point>>>& rhs){return lhs.second.first < rhs.second.first; });
	//(2)取出中位数
	std::pair<RotatedRect, std::pair<float, std::vector<Point>>> median_rect = rects_S1[cvCeil(rects_S1.size() / 2.0)];

	//3.二次筛选S(初筛后的再按照宽度(median - margin,median + margin)进行复筛)
	const float margin = median_rect.second.first * 0.5;
	std::pair<double, double> width0_range = std::make_pair(median_rect.second.first - margin, median_rect.second.first + margin);
	rects_S1.erase(std::remove_if(rects_S1.begin(), rects_S1.end(),
		[width0_range](const std::pair<RotatedRect, std::pair<float, std::vector<Point>>>& rect){return (!range_in(static_cast<double>(rect.second.first), width0_range)); }),
		rects_S1.end());

	double calc_angle;
	std::pair<RotatedRect, std::pair<float, std::vector<Point>>> min_distance_pt;
	std::vector<std::pair<RotatedRect, std::pair<float, std::vector<Point>>>> distance_S2; //<最小旋转矩形rotated,矩形center到图像中心距离,轮廓>
	{
		//计算S1中各中点到图像中心的欧式距离，找到距离最近的轮廓中心点(黑格)
		//求轮廓中心点到图像中心的距离
		for_each(rects_S1.begin(), rects_S1.end(), [center_img, &distance_S2](const std::pair<RotatedRect, std::pair<float, std::vector<Point>>>& rect)
		{
			distance_S2.push_back(make_pair(rect.first, make_pair(static_cast<float>(cv::norm(center_img - rect.first.center)), rect.second.second)));
		});
		//获取距离最近的点
		min_distance_pt = *std::min_element(distance_S2.begin(), distance_S2.end(), [](const std::pair<RotatedRect, std::pair<float, std::vector<Point>>>& lhs, const std::pair<RotatedRect, std::pair<float, std::vector<Point>>>& rhs)
		{return lhs.second.first < rhs.second.first; });

		//计算中心矩形的周围的8个距离最近点
		std::vector<std::pair<RotatedRect, std::pair<double, std::vector<Point>>>> rects_select;
		//选择距离中心最近点距离超过20的点(排除自身点和距离较近的干扰点)
		std::copy_if(rects_S1.begin(), rects_S1.end(), std::back_inserter(rects_select),
			[min_distance_pt](const std::pair<RotatedRect, std::pair<float, std::vector<Point>>>& element)
		{return  cv::norm(min_distance_pt.first.center - element.first.center) >20; });
		//计算其他点到最近点的距离
		for_each(rects_select.begin(), rects_select.end(), [min_distance_pt](std::pair<RotatedRect, std::pair<double, std::vector<Point>>>& rect)
		{
			rect.second.first = static_cast<double>(cv::norm(min_distance_pt.first.center - rect.first.center));
		});
		//排序
		sort(rects_select.begin(), rects_select.end(), [](const std::pair<RotatedRect, std::pair<double, std::vector<Point>>>& lhs, const std::pair<RotatedRect, std::pair<double, std::vector<Point>>>& rhs)
		{return lhs.second.first < rhs.second.first; });
		if (rects_select.size() < 8)
		{
			return ErrorCode::ERROR_CHART_QUALITY;
		}
		//判断8个点围成一圈
		//////
		//方法1:8方向向量匹配,带角度阈值
		//方法2:按照首尾角度排序，计算两两之间距离较小才算围成一圈
		std::vector<cv::Point2f> points_8;
		points_8.push_back(rects_select[0].first.center);
		points_8.push_back(rects_select[1].first.center);
		points_8.push_back(rects_select[2].first.center);
		points_8.push_back(rects_select[3].first.center);
		points_8.push_back(rects_select[4].first.center);
		points_8.push_back(rects_select[5].first.center);
		points_8.push_back(rects_select[6].first.center);
		points_8.push_back(rects_select[7].first.center);
		int thres_dis = 15;
		if (!is_Around(points_8, thres_dis, matShow_forSave, inParam.bDebug, min_distance_pt.first.center))
		{
			return ErrorCode::ERROR_CHART_QUALITY;
		}
		//选择4，5，6，7这4个点来计算倾斜角度
		std::vector<cv::Point2f> points;
		points.push_back(rects_select[4].first.center);
		points.push_back(rects_select[5].first.center);
		points.push_back(rects_select[6].first.center);
		points.push_back(rects_select[7].first.center);
		std::vector<std::pair<cv::Point2f, cv::Point2f>> groups;
		splitpoint(points, groups);
		cv::Point yuandian = groups[1].first.y > groups[1].second.y ? groups[1].first : groups[1].second;
		cv::Point other = groups[1].first.y < groups[1].second.y ? groups[1].first : groups[1].second;
		calc_angle = calculateAngleWithXAxis(other, yuandian);
		//转换成rotated rect的角度
		if (abs(calc_angle) < 90){
			calc_angle = -calc_angle;
		}
		else{
			calc_angle = 180 - calc_angle;
			calc_angle = -(90 - calc_angle);
		}
	}

	//4.求平均角度
	double mean_angle = std::accumulate(rects_S1.begin(), rects_S1.end(), 0.0, [](double sum, const std::pair<RotatedRect, std::pair<float, std::vector<Point>>>& rect)
	{return sum + rect.first.angle; }) / rects_S1.size();
	// 判断角度是否接近,超出，以计算的为准
	if (abs(mean_angle - calc_angle) > 3)
		mean_angle = calc_angle;

	//(1)求直线的斜率k
	double k = 0.0, redian = 0.0, angle = 0.0;
	if (abs(mean_angle) < 45.0)
	{
		mean_angle = abs(mean_angle);
		redian = CV_PI*(mean_angle) / 180.0;
		k = -tan(redian);
	}
	else
	{
		angle = mean_angle + 90.0;
		mean_angle = -angle;
		redian = CV_PI*(angle) / 180.0;
		k = tan(redian);
	}
	outRes->chart1_angle = std::make_tuple(mean_angle, 0.0, 0.0);


	Point2d begin_pt;
	Point2d end_pt;
	if (inParam.pts.size() == 0) //首次找点
	{
		auto min_distance_pt_x = min_distance_pt.first.center.x;
		auto min_distance_pt_y = min_distance_pt.first.center.y;

		//6.根据angle和最近点求过该点的直线L0，再上移和下移最近矩形height的一半距离得到另外两条直线L1，L2
		//(1)截距: b = y - kx
		double bias = min_distance_pt_y - k*min_distance_pt_x;
		//(2)求平行直线的偏置
		double bias_up = bias + median_rect.second.first / 2;
		double bias_low = bias - median_rect.second.first / 2;
		//(3)求在直线之间的轮廓矩形
		std::vector<std::pair<RotatedRect, std::pair<float, std::vector<Point>>>> inner_point;
		std::copy_if(distance_S2.begin(), distance_S2.end(), std::back_inserter(inner_point),
			[bias_low, bias_up, k](const std::pair<RotatedRect, std::pair<float, std::vector<Point>>>& element)
		{return  range_in(static_cast<double>(element.first.center.y), make_pair(k*element.first.center.x + bias_low, k*element.first.center.x + bias_up)); });

		////////////////////////////////////// 更新矩形中心为质心//////////////////////////////////////////////
		if (inParam.centroid == Centroid::CENTROID)
		{
			for (auto& ele : inner_point)
			{
				cv::Moments mu = moments(ele.second.second, false);
				cv::Point2f mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
				ele.first.center = mc;
			}
		}
		///////////////////////////////////////////////////////////////////////////////////////////////////////////

		//7.X坐标升序,求解6个相邻中心点距离和
		sort(inner_point.begin(), inner_point.end(), [](const std::pair<RotatedRect, std::pair<float, std::vector<Point>>>& lhs, const std::pair<RotatedRect, std::pair<float, std::vector<Point>>>& rhs)
		{return lhs.first.center.x < rhs.first.center.x; });
		for (int i = 0; i < inner_point.size(); i++)
		{
			circle(matShow_forSave, inner_point.at(i).first.center + Point2f(0.0f, height / 4), 10, Scalar(255, 255, 255), 2, 8);
			log_function(cv::format("Center coordinates of black rect:(%.6lf,%.6lf)", inner_point.at(i).first.center.x + 0.0f, inner_point.at(i).first.center.y + height / 4));
		}
		if (inner_point.size() < 6){

			log_function("Counts of black rect less than 6");
			return ErrorCode::ERROR_CONTOUR_COUNTS;
		}

		//8.判断抓取的点是否连续
		std::vector<std::pair<RotatedRect, std::pair<float, std::vector<Point>>>> clac_pt(inner_point.begin() + static_cast<int>(cvCeil(inner_point.size() / 2.0)) - 3, inner_point.begin() + static_cast<int>(cvCeil(inner_point.size() / 2.0)) + 3);
		std::vector<std::pair<RotatedRect, float>>  clac_pt_in;
		for (const auto& ele : clac_pt){
			clac_pt_in.push_back(std::make_pair(ele.first, ele.second.first));
		}
		if (!is_Continous(clac_pt_in, median_rect.second.first, matShow_forSave, inParam.bDebug))
		{
			return ErrorCode::ERROR_CHART_QUALITY;
		}

		end_pt = (*(inner_point.begin() + static_cast<int>(cvCeil(inner_point.size() / 2.0)) + 2)).first.center + Point2f(0.0f, height / 4);
		begin_pt  = (*(inner_point.begin() + static_cast<int>(cvCeil(inner_point.size() / 2.0)) - 3)).first.center + Point2f(0.0f, height / 4);
		line(matShow_forSave, begin_pt, end_pt, Scalar(255, 255, 255), 3);
		outRes->pts.emplace_back(make_pair(begin_pt, end_pt));
		//输出连续计算距离的点，为下一次计算准备
		for (auto it = (inner_point.begin() + static_cast<int>(cvCeil(inner_point.size() / 2.0)) - 3); it != (inner_point.begin() + static_cast<int>(cvCeil(inner_point.size() / 2.0)) + 3);it++)
		{
			outRes->pts_cb.push_back((*it).first.center + Point2f(0.0, height / 4));
		}
	}
	else // 已经存在点，查找与当前点最近的点
	{
		////////////////////////////////////// 更新矩形中心为质心//////////////////////////////////////////////
		if (inParam.centroid == Centroid::CENTROID)
		{
			for (auto& ele : rects_S1)
			{
				cv::Moments mu = moments(ele.second.second, false);
				cv::Point2f mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
				ele.first.center = mc;
			}
		}
		//查找距离已知点距离最近的点
		std::vector<std::pair<RotatedRect, float>>  clac_pt_in;
		for (const auto& pt : inParam.pts)
		{
			std::vector<std::pair<RotatedRect, std::pair<float, std::vector<Point>>>> distance_S3; //<最小旋转矩形rotated,矩形center到图像中心距离,轮廓>
			for_each(rects_S1.begin(), rects_S1.end(), [height,pt, &distance_S3](const std::pair<RotatedRect, std::pair<float, std::vector<Point>>>& rect)
			{
				distance_S3.push_back(make_pair(rect.first, make_pair(static_cast<float>(cv::norm(cv::Point2f(pt.x, pt.y) - (rect.first.center + Point2f(0.0f, height / 4)))), rect.second.second)));
			});

			//距离排序
			sort(distance_S3.begin(), distance_S3.end(), [](const std::pair<RotatedRect, std::pair<float, std::vector<Point>>>& lhs, const std::pair<RotatedRect, std::pair<float, std::vector<Point>>>& rhs)
			{return lhs.second.first < rhs.second.first; });
			//找到最近点
			auto nearst_pt = distance_S3.front();
			clac_pt_in.push_back(std::make_pair(nearst_pt.first, nearst_pt.second.first));
			outRes->pts_cb.push_back(nearst_pt.first.center + Point2f(0.0f, height / 4));
		}

		//8.判断抓取的点是否连续
		if (!is_Continous(clac_pt_in, median_rect.second.first, matShow_forSave, inParam.bDebug))
		{
			return ErrorCode::ERROR_CHART_QUALITY;
		}

		begin_pt = outRes->pts_cb.front();
		end_pt = outRes->pts_cb.back();
		line(matShow_forSave, begin_pt, end_pt, Scalar(255, 255, 255), 3);
		outRes->pts.emplace_back(make_pair(begin_pt, end_pt));
	}


	if (inParam.bDebug)
	{
		imwrite("ChessBoardChart.tiff", matShow_forSave);
	}
	//坐标距离
	double width_5_dis = cv::norm(begin_pt - end_pt);
	outRes->crop_width = width_5_dis;
	return ErrorCode::ERROR_OK;
}