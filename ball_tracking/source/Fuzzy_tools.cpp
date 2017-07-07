#include "Fuzzy_tools.h"

Fuzzy::Fuzzy(){
  distant = 200;
  middle = 100;
  close = 0;
}

Fuzzy::~Fuzzy(){

}

double Fuzzy::toKP(double devPoint){
  double kp;

  kp = FuzzyGrade(devPoint, middle, distant) * 0.01 +
    FuzzyTriangleGrade(devPoint, close, middle, distant) * 0.005 +
    FuzzyReverseGrade(devPoint, close, middle) * 0.0001;//...みたいな感じ

  return kp;

}

// 傾斜型メンバーシップのグレード値計算関数
// 入力：value 左下：x0 右上：x1
double Fuzzy::FuzzyGrade(double value, double x0, double x1){
	double result = 0;
	double x;

	x = value;

	if(x <= x0)
		result = 0;
	else if(x >= x1)
		result = 1;
	else
		result = (x - x0) / (x1 - x0);

	return result;
}

// 逆傾斜型メンバーシップのグレード値計算関数
// 入力：value 左上：x0 右下：x1
double Fuzzy::FuzzyReverseGrade(double value, double x0, double x1){
	double result = 0;
	double x;

	x = value;

	if(x <= x0)
		result = 1;
	else if(x >= x1)
		result = 0;
	else
		result = (x1 - x )/(x1 - x0);

	return result;
}

// 三角型メンバーシップのグレード値計算関数
// 入力：value 左下：x0 中央上：x1 右下：x2
double Fuzzy::FuzzyTriangleGrade(double value, double x0, double x1, double x2){
	double result = 0;
	double x;

	x = value;

	if(x <= x0 || x >= x2)
		result = 0;
	else if(x == x1)
		result = 1;
	else if((x > x0) && (x < x1))
		result = (x - x0) / (x1 - x0);
	else
		result = (x2 - x) / (x2 - x1);

	return result;
}
