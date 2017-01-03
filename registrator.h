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

// 读取数据
bool ReadFiles(int file_num);

// 全部注册函数
bool RegistrationAll();

// 保存模型数据
bool SaveModel(string model, string file_name);

// 对齐模型
bool RegistrationModel(int model_num);

// 合并模型
bool CombineModel();

// 寻找最近点
uint FindClosestPointInAddition(uint point_idx);
uint FindClosestPointInModel(uint point_idx);

#endif
