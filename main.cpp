#include "registrator.h"


const int kControlPointNum = 1000;

bool ReadFiles(int file_num)
{
	bool status = true;

	// �ļ�·��
	string input_file_path = "Data/";
	string input_file_name = "";
	string input_file_suffix = ".asc";
	
	// ��ȡ�ļ�
	for (int idx = 0; idx < file_num; idx++)
	{
		// ����·��
		string idx2str;
		stringstream ss;
		ss << idx + 1;
		ss >> idx2str;
		string file_name = input_file_path
			+ input_file_name
			+ idx2str
			+ input_file_suffix;

		// ��ȡ����������
		vector<Point3f> my_cloud;
		fstream file;
		file.open(file_name);
		char tmp[256];
		file.getline(tmp, 0xff, '\n');	// ����ǰ����
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

	// ��ʼ����ز���
	g_model = g_point_cloud[0];

	int len = g_point_cloud.size();
	for (int idx = 1; idx < len; idx++)
	{
		// ��׼
		if (status)
		{
			g_addition = g_point_cloud[1];
			status = RegistrationModel();
		}
		// �ϲ�
		if (status)
		{
			status = CombineModel();
		}
	}

	return status;
}

bool SaveModel() // unfinished
{
	bool status = true;

	return status;
}

bool RegistrationModel() // unfinished
{
	bool status = true;

	// ���ѡȡ���Ƶ�
	srand(clock());
	g_model_control.clear();
	int data_size = g_model.size();
	for (int idx = 0; idx < kControlPointNum; idx++)
	{
		uint point_idx = (rand() * 1000) % data_size;
		g_model_control.push_back(point_idx);
	}

	// ����kd-tree
	Mat model_mat(g_model);
	model_mat = model_mat.reshape(1);
	g_model_tree.release();
	g_model_tree.build(model_mat, flann::KDTreeIndexParams(4));
	Mat addition_mat(g_addition);
	addition_mat = addition_mat.reshape(1);
	g_addition_tree.release();
	g_addition_tree.build(g_addition, flann::KDTreeIndexParams(4));

	// ��������
	while (true)
	{
		// ѡ����ԵĶ�Ӧ��
		for (int idx = 0; idx < kControlPointNum; idx++)
		{
			uint model_idx = g_model_control[idx];
			uint addition_idx = FindClosestPoint(model_idx);
		}

		// ������ת��ƽ������
		

		// ����ģ��
	}
	
	return status;
}

bool CombineModel() // unfinished
{
	bool status = true;

	// ��򵥵�ֱ�Ӽ���

	return status;
}

uint FindClosestPoint(uint point_idx)
{
	uint close_idx = 0;

	// ֱ������������
	Point3f pt = g_model[point_idx];
	vector<float> query;
	query.push_back(pt.x);
	query.push_back(pt.y);
	query.push_back(pt.z);

	vector<int> addition_idx;
	vector<float> addition_dists;

	g_addition_tree.knnSearch(query, addition_idx, addition_dists, 1);

	return (uint)addition_idx[0];
}

int main()
{
	bool status = true;

	// ��ȡ���ݵ��ڴ�
	if (status)
	{
		printf("Reading files...");
		status = ReadFiles(2);
		printf("Finished.\n");
	}
	
	// ����׼
	if (status)
	{
		printf("Registration:\n");
		status = RegistrationAll();
		printf("Finished.\n");
	}

	// �������ս��
	if (status)
	{
		status = SaveModel();
	}

	return 0;
}