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
// BIpoint mousePoint = {-1, -1};
BIpoint x_init = {-1, -1};
BIpoint x_goal = {-1, -1};

void onMouse(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        updata = true;
        x_init.x = x;
        x_init.y = y;
        std::cout << "Left button of the mouse is clicked - x_init (" << x << ", " << y << ")" << std::endl;
    }
    if (event == cv::EVENT_RBUTTONDOWN)
    {
        updata = true;
        x_goal.x = x;
        x_goal.y = y;
        std::cout << "Right button of the mouse is clicked - x_goal (" << x << ", " << y << ")" << std::endl;
    }
}

// 前提：启动approx_work.py程序
// Premise: Launch the approx_work.py program

// 左点击设置x_init
// 右点击设置x_goal
// 空格启动CDT-kSNP规划器，命令行输入k值
// ESC键退出
// Left click to set x_init 
// Right click to set x_goal 
// Space key to start CDT-kSNP planner, input k value in command line 
// ESC key to exit
int main()
{
    cv::namedWindow("Video", 0);
    cv::resizeWindow("Video", 1200, 1200);
    cv::setMouseCallback("Video", onMouse);

    BImap cemap;
    kSNPPtask task;

    int64_t t0 = utime_ns();
    cemap.MaptoBInavi((char *)"./expMap/GAME.png", 0.1, 0.9);
    int64_t t1 = utime_ns();
    std::cout << "Itime(ns): " << (t1 - t0) << std::endl;
    cemap.DrawBImap(true);

    int32_t cvkey = '0';
    while (cvkey != 27)
    {
        cv::Mat image = cemap.BIdmap.clone();;
        if (x_init.x >= 0)
            cv::circle(image, cv::Point((int)(x_init.x), (int)(x_init.y)), 3, cv::Scalar(255, 0, 0), -1);
        if (x_goal.x >= 0)
            cv::circle(image, cv::Point((int)(x_goal.x), (int)(x_goal.y)), 3, cv::Scalar(0, 0, 255), -1);
        cv::imshow("Video", image);
        cvkey = cv::waitKey(30);

        if (cvkey == ' ')
        {
            int32_t k = 0;
            std::cout << "Input k:";
            std::cin >> k;
            if (k <= 0 || x_init.x < 0 || x_goal.x < 0)
                continue;
            cemap.kSNPPlanner(task, x_init, x_goal, k);
            for (int32_t i = 0; i < task.Q_kSNP.size(); i++)
            {
                image = cemap.BIdmap.clone();
                printf("Cost k%03d: %7.2lf\r\n", i + 1, task.Q_Costs[i]);
                drawBIpath(task.Q_kSNP[i], image, 10, cv::Scalar(0, 0, 255));
            }
        }
    }

    return 0;
}
