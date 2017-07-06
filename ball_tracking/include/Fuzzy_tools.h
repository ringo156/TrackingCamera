/// Fuzzy Tool Header Y.HOSHINO KUT 2015/10/07

#ifndef _FUZZY_TOOLS_H_
#define _FUZZY_TOOLS_H_

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <time.h>
#include <sys/time.h>
#include <iostream>
#include <stdint.h>

class Fuzzy{
  private:
    double distant;
    double middle;
    double close;

  public:
    Fuzzy();
    ~Fuzzy();

    double toKP(double devPoint);//x,y別々で関数を呼び出させる sqrtで出したほうがいいのでは？？
    // 傾斜型メンバーシップのグレード値計算関数
    // 入力：value 左下：x0 右上：x1
    double FuzzyGrade(double value, double x0, double x1);
    // 逆傾斜型メンバーシップのグレード値計算関数
    // 入力：value 左上：x0 右下：x1
    double FuzzyReverseGrade(double value, double x0, double x1);
    // 三角型メンバーシップのグレード値計算関数
    // 入力：value 左下：x0 中央上：x1 右下：x2
    double FuzzyTriangleGrade(double value, double x0, double x1, double x2);

};

#endif
