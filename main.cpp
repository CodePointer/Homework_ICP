#include "registrator.h"


const int kControlPointNum = 2000;
const int kMaxIterationNum = 40;
const float kVeryBigFloatNum = 9999999999.0;
const float kEpsilonForErrorFunction = 10.0;
const float kMergeEpsilon = 1.0;

bool ReadFiles(int file_num)
{
	bool status = true;

	// 文件路径
	string input_file_path = "Data/";
	string input_file_name = "";
	string input_file_suffix = ".asc";
	
	// 读取文件
	for (int idx = 0; idx < file_num; idx++)
	{
		// 创建路径
		string idx2str;
		stringstream ss;
		ss << idx + 1;
		ss >> idx2str;
		string file_name = input_file_path
			+ input_file_name
			+ idx2str
			+ input_file_suffix;

		// 读取数据至点云
		vector<Point3f> my_cloud;
		fstream file;
		file.open(file_name);
		char tmp[256];
		file.getline(tmp, 0xff, '\n');	// 跳过前两行
		file.getline(tmp, 0xff, '\n');
		int k = 0;
		while (!file.eof())
		{
			k++;
			Point3f pt(0, 0, 0);
			file >> pt.x;
			file >> pt.y;
			file >> pt.z;
			my_cloud.push_back(pt);
		}
		file.close();
		g_point_cloud.push_back(my_cloud);

		printf(".");
	}

	return status;
}

bool RegistrationAll()
{
	bool status = true;

	// 初始化相关参数
	g_model = g_point_cloud[0];

	int len = g_point_cloud.size();
	for (int idx = 1; idx < len; idx++)
	{
		// 配准
		if (status)
		{
			g_addition.clear();
			g_addition = g_point_cloud[idx];
			status = RegistrationModel(idx);
		}
		// 合并
		if (status)
		{
			status = CombineModel();
		}
		else
		{
			printf("\tCombining failed. Continue to next frame.\n");
			status = true;
		}
	}

	return status;
}

bool SaveModel(string model, string file_name)
{
	bool status = true;

	// 输出路径


	if (model == "model")
	{
		size_t point_size = g_model.size();
		fstream file;
		file.open(file_name, ios::out);
		if (!file)
		{
			status = false;
		}
		else
		{
			for (int i = 0; i < point_size; i++)
			{
				Point3f pt = g_model[i];
				file << pt.x << " ";
				file << pt.y << " ";
				file << pt.z << endl;
			}
			file.close();
		}
	}
	else if (model == "addition")
	{
		size_t point_size = g_addition.size();
		fstream file;
		file.open(file_name, ios::out);
		if (!file)
		{
			status = false;
		}
		else
		{
			for (int i = 0; i < point_size; i++)
			{
				Point3f pt = g_addition[i];
				file << pt.x << " ";
				file << pt.y << " ";
				file << pt.z << endl;
			}
			file.close();
		}
	}
	else
	{
		status = false;
	}

	return status;
}

bool RegistrationModel(int model_num)
{
	bool status = true;

	// 随机选取控制点
	srand(clock());
	g_model_control.clear();
	int data_size = g_model.size();
	for (int idx = 0; idx < kControlPointNum; idx++)
	{
		uint point_idx = (rand() * 1000) % data_size;
		g_model_control.push_back(point_idx);
	}

	// 构建kd-tree
	Mat model_mat(g_model);
	model_mat = model_mat.reshape(1);
	g_model_tree.release();
	g_model_tree.build(model_mat, flann::KDTreeIndexParams(4));
	Mat addition_mat(g_addition);
	addition_mat = addition_mat.reshape(1);
	g_addition_tree.release();
	g_addition_tree.build(addition_mat, flann::KDTreeIndexParams(4));

	// 迭代过程
	float E2_now = -kVeryBigFloatNum ;
	float E2_before = kVeryBigFloatNum;
	int iter_k = 0;
	while ((abs(E2_before - E2_now) >= kEpsilonForErrorFunction) && (iter_k < kMaxIterationNum))
	{
		iter_k++;
		printf("\tIteration %d-%d begin...", model_num, iter_k);

		// 选择相对的对应点
		g_addition_control.clear();
		for (int idx = 0; idx < kControlPointNum; idx++)
		{
			uint model_idx = g_model_control[idx];
			uint addition_idx = FindClosestPointInAddition(model_idx);
			g_addition_control.push_back(addition_idx);
		}

		// 估计平移向量
		Point3f p_model_mean = (0, 0, 0);
		Point3f p_addition_mean = (0, 0, 0);
		for (int idx = 0; idx < kControlPointNum; idx++)
		{
			p_model_mean += g_model[g_model_control[idx]];
			p_addition_mean += g_addition[g_addition_control[idx]];
		}
		p_model_mean = p_model_mean / kControlPointNum;
		p_addition_mean = p_addition_mean / kControlPointNum;
		Mat trans_vec = Mat(p_model_mean - p_addition_mean).reshape(1);

		// 估计旋转四元数与误差E2
		Mat A_mat[kControlPointNum];
		Mat B_mat;
		B_mat.create(4, 4, CV_32FC1);
		B_mat.setTo(0);
		for (int i = 0; i < kControlPointNum; i++)
		{
			A_mat[i].create(4, 4, CV_32FC1);
			Point3f d_i = g_addition[g_addition_control[i]] - p_addition_mean;
			Point3f d_i_p = g_model[g_model_control[i]] - p_model_mean;
			A_mat[i].setTo(0);
			A_mat[i].at<float>(0, 1) = d_i.x - d_i_p.x;
			A_mat[i].at<float>(0, 2) = d_i.y - d_i_p.y;
			A_mat[i].at<float>(0, 3) = d_i.z - d_i_p.z;
			A_mat[i].at<float>(1, 0) = d_i_p.x - d_i.x;
			A_mat[i].at<float>(2, 0) = d_i_p.y - d_i.y;
			A_mat[i].at<float>(3, 0) = d_i_p.z - d_i.z;
			A_mat[i].at<float>(1, 2) = (d_i.z + d_i_p.z);
			A_mat[i].at<float>(1, 3) = -(d_i.y + d_i_p.y);
			A_mat[i].at<float>(2, 1) = -(d_i.z + d_i_p.z);
			A_mat[i].at<float>(2, 3) = (d_i.x + d_i_p.x);
			A_mat[i].at<float>(3, 1) = (d_i.y + d_i_p.y);
			A_mat[i].at<float>(3, 2) = -(d_i.x + d_i_p.x);


			B_mat = B_mat + A_mat[i] * A_mat[i].t();
			/*cout << B_mat << endl;
			if (i == 500)
			{
				cout << B_mat << endl;;
			}*/
		}
		//cout << B_mat << endl;
		Mat eValuesMat;
		Mat eVectorsMat;
		eigen(B_mat, eValuesMat, eVectorsMat);
		/*cout << B_mat << endl;
		cout << eValuesMat << endl;
		cout << eVectorsMat << endl;*/
		float min = kVeryBigFloatNum;
		int min_idx = -1;
		for (int i = 0; i < 4; i++)
		{
			float value = (float)eValuesMat.at<float>(i, 0);
			if (abs(value) < min)
			{
				min = abs(value);
				min_idx = i;
			}
		}
		E2_before = E2_now;
		E2_now = min;
		Mat rotation_4vec = eVectorsMat.row(min_idx).t();

		// 估计旋转矩阵
		float theta = acos(rotation_4vec.at<float>(0, 0)) * 2;
		Mat omega_vec = rotation_4vec.rowRange(1, 4) / sin(theta / 2);
		/*cout << rotation_4vec << endl;
		cout << theta << endl;
		cout << omega_vec << endl;*/
		float w1 = omega_vec.at<float>(0, 0);
		float w2 = omega_vec.at<float>(1, 0);
		float w3 = omega_vec.at<float>(2, 0);
		Mat rotation_mat;
		rotation_mat.create(3, 3, CV_32FC1);
		rotation_mat.at<float>(0, 0) = cos(theta) + w1*w1*(1 - cos(theta));
		rotation_mat.at<float>(0, 1) = w1*w2*(1 - cos(theta)) - w3*sin(theta);
		rotation_mat.at<float>(0, 2) = w1*w3*(1 - cos(theta) + w2*sin(theta));
		rotation_mat.at<float>(1, 1) = cos(theta) + w2*w2*(1 - cos(theta));
		rotation_mat.at<float>(1, 2) = w2*w3*(1 - cos(theta)) - w1*sin(theta);
		rotation_mat.at<float>(2, 2) = cos(theta) + w3*w3*(1 - cos(theta));
		rotation_mat.at<float>(1, 0) = rotation_mat.at<float>(0, 1);
		rotation_mat.at<float>(2, 0) = rotation_mat.at<float>(0, 2);
		rotation_mat.at<float>(2, 1) = rotation_mat.at<float>(1, 2);
		/*cout << rotation_mat << endl;*/


		// 修正模型
		vector<Point3f> tmp_vec(g_addition);
		g_addition.clear();
		size_t point_size = tmp_vec.size();
		for (int i = 0; i < point_size; i++)
		{
			Point3f pt = tmp_vec[i];
			Mat point_vec;
			point_vec.create(3, 1, CV_32FC1);
			point_vec.at<float>(0, 0) = pt.x;
			point_vec.at<float>(1, 0) = pt.y;
			point_vec.at<float>(2, 0) = pt.z;
			point_vec = point_vec + trans_vec;
			Mat new_point_vec = rotation_mat * point_vec;
			Point3f new_pt(0, 0, 0);
			new_pt.x = new_point_vec.at<float>(0, 0);
			new_pt.y = new_point_vec.at<float>(1, 0);
			new_pt.z = new_point_vec.at<float>(2, 0);
			g_addition.push_back(new_pt);
		}

		printf("finished. With deltaE2 = %f\n", abs(E2_now-E2_before));

		// 保存模型
		stringstream ss;
		string idx2str;
		string num2str;
		ss << iter_k;
		ss >> idx2str;
		ss.clear();
		ss << model_num;
		ss >> num2str;
		if (iter_k % 10 == 0)
		{
			SaveModel("addition", "model_" + num2str + "_iter_" + idx2str + ".asc");
		}
	}
	
	// 保存最终迭代结果
	stringstream ss;
	string idx2str;
	string num2str;
	ss << iter_k;
	ss >> idx2str;
	ss.clear();
	ss << model_num;
	ss >> num2str;
	SaveModel("addition", "f_model_" + num2str + "_iter_" + idx2str + ".asc");

	// 判断是否成功融合（错误率较小）
	printf("\tE2 = %f\n", E2_now);
	if (E2_now >= 20000)
	{
		status = false;
	}

	return status;
}

bool CombineModel() // unfinished
{
	bool status = true;

	// 最简单的直接加入
	printf("\tCombining model(%d)", g_addition.size());
	vector<Point3f> tmp_points;
	for (int i = 0; i < g_addition.size(); i++)
	{
		uint idx_addition = i;
		uint idx_model = FindClosestPointInModel(idx_addition);
		Point3f dist_pf = g_model[idx_model] - g_addition[idx_addition];
		float dist = (dist_pf.x*dist_pf.x) + (dist_pf.y*dist_pf.y) + (dist_pf.z*dist_pf.z);
		if (dist > kMergeEpsilon)
		{
			tmp_points.push_back(g_addition[idx_addition]);
		}
		if (i % 10000 == 0)
		{
			printf(".");
		}
	}

	for (int i = 0; i < tmp_points.size(); i++)
	{
		g_model.push_back(tmp_points[i]);
	}
	g_addition.clear();
	printf("finished.(%d points)\n", tmp_points.size());

	return status;
}

uint FindClosestPointInAddition(uint point_idx)
{
	uint close_idx = 0;

	// 直接找最近距离点
	/*Point3f pt = g_model[point_idx];
	vector<float> query;
	query.push_back(pt.x);
	query.push_back(pt.y);
	query.push_back(pt.z);

	vector<int> addition_idx;
	vector<float> addition_dists;

	g_addition_tree.knnSearch(query, addition_idx, addition_dists, 1);

	return (uint)addition_idx[0];*/

	/// 寻找沿法线方向最近点
	Point3f pt = g_model[point_idx];
	vector<float> query;
	query.push_back(pt.x);
	query.push_back(pt.y);
	query.push_back(pt.z);
	vector<int> addition_idx;
	vector<float> addition_dists;
	g_addition_tree.knnSearch(query, addition_idx, addition_dists, 20);
	vector<int> model_idx;
	vector<float> model_dists;
	g_model_tree.knnSearch(query, model_idx, model_dists, 20);

	// 对于每个addition点，查找其与周围点的点积和
	float min_value = kVeryBigFloatNum;
	uint min_idx = 0;
	for (int i = 0; i < addition_idx.size(); i++)
	{
		Point3f addition_point = g_addition[addition_idx[i]];
		Point3f addition_vec = addition_point - pt;
		float value = 0;
		for (int j = 0; j < model_idx.size(); j++)
		{
			Point3f model_point = g_model[model_idx[j]];
			Point3f model_vec = model_point - pt;
			value += abs(model_vec.x*addition_vec.x 
				+ model_vec.y*addition_vec.y 
				+ model_vec.z*addition_vec.z);
		}
		if (value < min_value)
		{
			min_value = value;
			min_idx = i;
		}
	}
	/*if (min_idx != 0)
	{
		printf("min_idx=%d\n", min_idx);
	}*/
	return addition_idx[min_idx];
}

uint FindClosestPointInModel(uint point_idx)
{
	uint close_idx = 0;

	// 直接找最近距离点
	Point3f pt = g_addition[point_idx];
	vector<float> query;
	query.push_back(pt.x);
	query.push_back(pt.y);
	query.push_back(pt.z);

	vector<int> model_idx;
	vector<float> model_dists;

	g_model_tree.knnSearch(query, model_idx, model_dists, 1);

	return (uint)model_idx[0];
}

int main()
{
	bool status = true;

	// 读取数据到内存
	if (status)
	{
		printf("Reading files...");
		status = ReadFiles(7);
		printf("Finished.\n");
	}
	
	// 总配准
	if (status)
	{
		printf("Registration:\n");
		status = RegistrationAll();
		printf("Finished.\n");
	}

	// 保存最终结果
	if (status)
	{
		printf("Saving model...");
		status = SaveModel("model", "model.asc");
		printf("Finished.\n");
	}

	system("PAUSE");

	return 0;
}