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

struct testSNPPtask
{
    char *name;
    BIpoint x_init;
    BIpoint x_goal;
};

std::vector<testSNPPtask> testTasks = {
    {(char *)"./expMap/MESS.png", {1000, 1125}, {200, 150}},
    {(char *)"./expMap/MAZE.png", {1100, 2200}, {2360, 360}},
    {(char *)"./expMap/GAME.png", {180, 320}, {2400, 2350}},
    {(char *)"./expMap/FLOOR.png", {800, 1900}, {1300, 170}},
};

int32_t kNUM[] = {5, 10, 25, 50};

// 前提：启动approx_work.py程序
// Premise: Launch the approx_work.py program

// 子节IV A. Planning Efficiency for k-SNPP in the Simulation中的实验程序
// Experimental Program in Subsection IV A. Planning Efficiency for k-SNPP in the Simulation
int main()
{

    int32_t map_index = 0;
    std::cout << "Input map_index:";
    std::cin >> map_index;
    if (map_index >= testTasks.size())
        return 1;
    testSNPPtask &testTask = testTasks[map_index];

    BImap cemap;
    kSNPPtask task;
    int64_t t0 = utime_ns();
    cemap.MaptoBInavi(testTask.name, 0.1, 0.9);
    int64_t t1 = utime_ns();
    std::cout << "Itime(ns): " << (t1 - t0) << std::endl;
    cemap.DrawBImap(true);

    cv::Mat image;
    int count = 100;
    for (size_t k = 0; k < 4; k++)
    {
        t0 = utime_ns();
        for (size_t i = 0; i < count; i++)
            cemap.kSNPPlanner(task, testTask.x_init, testTask.x_goal, kNUM[k]); //, false
        t1 = utime_ns();
        std::cout << k << " time(ns): " << (t1 - t0) / count << std::endl;
        // image = cemap.BIdmap.clone();
        // drawBIpath(task.Q_kSNP[0], image, 0, cv::Scalar(0, 0, 255));
        // cv::imshow("Video", image);
        // cv::waitKey(0);
    }

    return 0;
}
