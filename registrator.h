#ifndef _REGISTRATOR_
#define _REGISTRATOR_

#define _CRT_SECURE_NO_WARNINGS

// opencv, flann
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/flann/flann.hpp>

// C++ lib
#include <vector>
#include <iostream>
#include <string>
#include <strstream>
#include <fstream>

// For rand
#include <ctime>

// namespace
using namespace std;
using namespace cv;

// parameters
extern const int kControlPointNum;
extern const float kVeryBigFloatNum;

// global varibles
vector<vector<Point3f>> g_point_cloud;
vector<Point3f> g_model;
vector<Point3f> g_addition;
flann::Index g_model_tree;
flann::Index g_addition_tree;
vector<uint> g_model_control;
vector<uint> g_addition_control;

// ��ȡ����
bool ReadFiles(int file_num);

// ȫ��ע�ắ��
bool RegistrationAll();

// ����ģ������
bool SaveModel(string model, string file_name);

// ����ģ��
bool RegistrationModel(int model_num);

// �ϲ�ģ��
bool CombineModel();

// Ѱ�������
uint FindClosestPointInAddition(uint point_idx);
uint FindClosestPointInModel(uint point_idx);

#endif
