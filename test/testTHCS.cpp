#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <random>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "CDTmap.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

// 回调函数，当鼠标左键按下时调用
int32_t updata = false;
BIpoint mousePoint = {-1, -1};
void onMouse(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        updata = true;
        mousePoint.x = x;
        mousePoint.y = y;
        // 在控制台输出鼠标左击位置的坐标
        std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
}

// 前提：启动approx_work.py程序
// Premise: Launch the approx_work.py program

// 子节IV D. Expanding Application Experiment中的实验程序
// Experimental Program in Subsection IV D. Expanding Application Experiment
int main()
{
    cv::namedWindow("Video", 0);
    cv::resizeWindow("Video", 1200, 1200);
    cv::setMouseCallback("Video", onMouse);

    BImap cemap;
    kSNPPtask task;

    cemap.MaptoBInavi((char *)"/home/tzyh_subsys/WorkSpace/CDTkSNPP/expMap/realMap2_Dil.png", 0.1, 0.9);
    cemap.DrawBImap(true);

    double re = 570.0 / 1299; // pix to centimeter
    BIpoint x_star = {135 / re, 495 / re};
    BIpoint x_goal = {555 / re, 195 / re};
    // BIpoint x_goal = {435 / re,  75 / re};

    cemap.kSNP2THCSearcher(task, x_star, x_goal, 850 / re, 9999999);

    cv::Mat image;
    for (int32_t i = 0; i < task.Q_kSNP.size(); i++)
    {
        image = cemap.BIdmap.clone();

        printf("Cost k%02d: %7.2lf\r\n", i + 1, task.Q_Costs[i] * re / 100);
        drawBIpath(task.Q_kSNP[i], image, 10, cv::Scalar(0, 0, 255));
        cv::imshow("Video", image);
        cv::waitKey(10);
    }
    cv::waitKey(0);

    return 0;
}
