#include "CDTmap.h"

void drawBIgraphObs(BIgraph &graph, cv::Mat image)
{

    for (const BIobspolygon &obspolygon : graph.obspolygonList)
    {
        cv::Mat Img = image.clone();
        BIfreepolygon *polyOld = &graph.freepolygonList[obspolygon.polygonRlist.back()];
        BIpoint coreOld = polyOld->core;
        for (const int32_t freepolyIndex : obspolygon.polygonRlist)
        {
            BIfreepolygon *polyNow = &graph.freepolygonList[freepolyIndex];
            BIpoint core = polyNow->core;
            BIpoint cutcore = graph.cutlineList[findCommonElement(polyOld->cutlinelink, polyNow->cutlinelink)].core;
            cv::Point cutPs = cv::Point((int)(coreOld.x), (int)(coreOld.y));
            cv::Point cutPo = cv::Point((int)(cutcore.x), (int)(cutcore.y));
            cv::Point cutPe = cv::Point((int)(core.x), (int)(core.y));
            cv::line(Img, cutPs, cutPo, cv::Scalar(127, 0, 127), 2);
            cv::line(Img, cutPo, cutPe, cv::Scalar(127, 0, 127), 2);
            polyOld = polyNow;
            coreOld = core;
        }
        cv::imshow("Video", Img);
        cv::waitKey();
    }
}

// 绘制多边形
void drawPolygons(const BIpolygons &polygons, cv::Mat image)
{

    for (const auto &polygon : polygons)
    {
        std::vector<cv::Point> cvPolygon;
        for (const auto &point : polygon)
        {
            cvPolygon.push_back(cv::Point(static_cast<int>(point.x), static_cast<int>(point.y)));
        }

        // 绘制多边形，这里假设所有多边形都是闭合的
        const cv::Point *pts = (const cv::Point *)cv::Mat(cvPolygon).data;
        int npts = cv::Mat(cvPolygon).rows;

        // 选择颜色、线宽等绘图参数
        cv::polylines(image, &pts, &npts, 1, true, cv::Scalar(127, 0, 127), 1);
    }
}

void drawConcaves(std::list<BIangle> &ConcaveSet, cv::Mat image)
{
    // 遍历ConcaveSet中的每个BIangle
    for (const auto &angle : ConcaveSet)
    {
        // 获取原点O的坐标
        cv::Point center(static_cast<int>(angle.O.x), static_cast<int>(angle.O.y));

        // 在图像上绘制圆形来标记原点O，这里选择半径为3，颜色为红色(0, 0, 255)，厚度为-1（实心圆）
        cv::circle(image, center, 3, cv::Scalar(0, 0, 255), -1);
    }
}
void drawAngle(BIangle &nowAngle, cv::Mat image)
{
    cv::Point cvPo = cv::Point((int)(nowAngle.O.x), (int)(nowAngle.O.y));
    cv::Point cvPs = cv::Point((int)(nowAngle.S.x), (int)(nowAngle.S.y));
    cv::Point cvPe = cv::Point((int)(nowAngle.E.x), (int)(nowAngle.E.y));
    cv::line(image, cvPo, cvPs, cv::Scalar(0, 255, 0), 2);
    cv::line(image, cvPo, cvPe, cv::Scalar(0, 255, 0), 2);
    cv::circle(image, cvPs, 3, cv::Scalar(255, 0, 0), -1);
    cv::circle(image, cvPe, 3, cv::Scalar(0, 0, 255), -1);
}

void drawBIgraph(BIgraph &graph, cv::Mat image)
{
    std::vector<BIcutline> &cutlineList = graph.cutlineList;
    std::vector<BIfreepolygon> &freepolygonList = graph.freepolygonList;

    for (const BIcutline &cutline : cutlineList)
    {
        cv::Point cutPs = cv::Point((int)(cutline.line.S.x), (int)(cutline.line.S.y));
        cv::Point cutPe = cv::Point((int)(cutline.line.E.x), (int)(cutline.line.E.y));

        cv::line(image, cutPs, cutPe, cv::Scalar(127, 0, 127), 1.5);
        cv::circle(image, cv::Point((int)(cutline.core.x), (int)(cutline.core.y)), 3, cv::Scalar(0, 0, 255), -1);
    }

    for (const BIfreepolygon &polygon : freepolygonList)
        cv::circle(image, cv::Point((int)(polygon.core.x), (int)(polygon.core.y)), 3, cv::Scalar(255, 0, 0), -1);
}


void drawBIring(std::list<BIpoint> &Path, cv::Mat &image)
{
    BIpoint pathPoint_old = *Path.begin();
    for (const BIpoint &pathPoint : Path)
    {
        cv::Point Sp, Ep;
        cv::Scalar color(255, 255, 255);
        color[0] = (int)(rand() / (double(RAND_MAX)) * 0);
        color[1] = (int)(rand() / (double(RAND_MAX)) * 255);
        color[2] = (int)(rand() / (double(RAND_MAX)) * 255);
        Sp = {(int)pathPoint.x, (int)pathPoint.y};
        Ep = {(int)pathPoint_old.x, (int)pathPoint_old.y};
        cv::line(image, Sp, Ep, color, 3);

        cv::imshow("Video", image);
        cv::waitKey(30);
        pathPoint_old = pathPoint;
    }
}


void drawBIpath(const std::list<BIpoint> &path, cv::Mat &image, int32_t waittime, cv::Scalar color)
{
    BIpoint pathPoint_old = path.front();
    for (const BIpoint &pathPoint : path)
    {
        cv::Point Sp, Ep;
        Sp = {(int)pathPoint.x, (int)pathPoint.y};
        Ep = {(int)pathPoint_old.x, (int)pathPoint_old.y};
        cv::line(image, Sp, Ep, color, 4);

        if (waittime)
        {
            cv::imshow("Video", image);
            cv::waitKey(waittime);
        }
        pathPoint_old = pathPoint;
    }
}


void drawBIfreeID(const BIgraph &graph, cv::Mat &image)
{
    const std::vector<BIfreepolygon> &freepolygonList = graph.freepolygonList;
    for (const BIfreepolygon &polygon : freepolygonList)
    {
        std::stringstream ss;
        ss << polygon.index;
        std::string text = ss.str();
        cv::putText(image, text, cv::Point(int(polygon.core.x), int(polygon.core.y)), cv::FONT_HERSHEY_COMPLEX,
                    1, cv::Scalar(255, 0, 0), 2);
    }
}