#include "CDTmap.h"

#include <math.h>

#include <list>
#include <fstream>
#include <nlohmann/json.hpp>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
using json = nlohmann::json;

// 从json到BIpoint的转换
void from_json(const json &j, BIpoint &p)
{
    p.x = j.at(0).get<double>();
    p.y = j.at(1).get<double>();
}

void BImap::MaptoBInavi(char *IMGmap, double ratio, double rsize, const char *addr, int port) // vector<vector<CSpoint>> &returnlist)
{
    mapratio = ratio;
    robotsize = rsize;

    json data;
    data["path"] = IMGmap;
    data["ratio"] = ratio;
    data["robotsize"] = rsize;
    data["thresh"] = 128;
    std::string pretty_str = data.dump();

    /***建立通讯***/
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0)
    {
        perror("socket");
        exit(1);
    }
    /* 将套接字和IP、端口绑定 */
    sockaddr_in addr_serv, addr_client;
    int socklen;
    memset(&addr_serv, 0, sizeof(sockaddr_in));   // 每个字节都用0填充
    memset(&addr_client, 0, sizeof(sockaddr_in)); // 每个字节都用0填充
    addr_serv.sin_family = AF_INET;               // 使用IPV4地址
    addr_serv.sin_port = htons(port);             // 端口
    /* INADDR_ANY表示不管是哪个网卡接收到数据，只要目的端口是SERV_PORT，就会被该应用程序接收到 */
    addr_serv.sin_addr.s_addr = inet_addr(addr);
    socklen = sizeof(addr_serv);

    int send_num = sendto(sock_fd, pretty_str.data(), pretty_str.size(), 0, (struct sockaddr *)&addr_serv, socklen);
    if (send_num < 0)
    {
        perror("sendto error:");
        exit(1);
    }
    char recv_buf[204800];
    int recv_num = recvfrom(sock_fd, recv_buf, 204799, 0, (struct sockaddr *)&addr_serv, (socklen_t *)&socklen);
    if (recv_num < 0)
    {
        perror("recvfrom error:");
        exit(1);
    }
    recv_buf[recv_num] = 0;
    close(sock_fd);
    /***解析数据***/
    json redata = json::parse(recv_buf);
    shapeX = redata[1][1];
    shapeY = redata[1][0];
    json &mapdata = redata[0];
    BIgraphList.resize(mapdata.size());
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(-1.0, 1.0);
    // int64_t t0 = utime_ns();
    for (int32_t i = 0; i < mapdata.size(); i++)
    {
        BIpolygons polygons;
        polygons = mapdata[i].get<BIpolygons>();
        StartCut(polygons, BIgraphList[i]);
    }
    // int64_t t1 = utime_ns();
    // std::cout << "ttime(ns): " << (t1 - t0) << std::endl;
}

void BImap::DrawBImap(bool debugmap)
{
    BIamap = cv::Mat::ones(shapeY, shapeX, CV_16U) * 0xFFFF;
    BIimap = cv::Mat::ones(shapeY, shapeX, CV_16U) * 0xFFFF;
    if (debugmap)
    {
        BIdmap = cv::Mat::zeros(shapeY, shapeX, CV_8UC3);
        BIfmap = cv::Mat::zeros(shapeY, shapeX, CV_8UC3);
    }
    for (int32_t graphIndex = 0; graphIndex < BIgraphList.size(); graphIndex++)
    {
        BIgraph &graph = BIgraphList[graphIndex];
        for (int32_t polyIndex = 0; polyIndex < graph.freepolygonList.size(); polyIndex++)
        {
            BIfreepolygon &freepolygon = graph.freepolygonList[polyIndex];
            BIpolygon &polygon = freepolygon.polygon;
            cv::Point PointArray[polygon.size()];
            for (int32_t pnum = 0; pnum < polygon.size(); pnum++)
            {
                PointArray[pnum].x = polygon[pnum].x;
                PointArray[pnum].y = polygon[pnum].y;
            }
            cv::fillConvexPoly(BIamap, PointArray, polygon.size(), cv::Scalar(graphIndex));
            cv::fillConvexPoly(BIimap, PointArray, polygon.size(), cv::Scalar(polyIndex));
            if (debugmap)
            {
                cv::Scalar color(255, 255, 255);
                cv::fillConvexPoly(BIfmap, PointArray, polygon.size(), color);
                color[0] = (int)(rand() / (double(RAND_MAX)) * 127) + 127;
                color[1] = (int)(rand() / (double(RAND_MAX)) * 127) + 127;
                color[2] = (int)(rand() / (double(RAND_MAX)) * 127) + 127;
                cv::fillConvexPoly(BIdmap, PointArray, polygon.size(), color);
            }
        }
        for (int32_t polyIndex = 0; polyIndex < graph.obspolygonList.size(); polyIndex++)
        {
            BIobspolygon &obspolygon = graph.obspolygonList[polyIndex];
            BIpolygon &polygon = obspolygon.polygon;
            std::vector<cv::Point> PointArray;
            PointArray.resize(polygon.size());
            for (int32_t pnum = 0; pnum < polygon.size(); pnum++)
            {
                PointArray[pnum].x = polygon[pnum].x;
                PointArray[pnum].y = polygon[pnum].y;
            }

            std::vector<std::vector<cv::Point>> contours;
            contours.push_back(PointArray);
            cv::fillPoly(BIimap, contours, cv::Scalar(polyIndex | 0xc000));
            if (debugmap)
                cv::fillPoly(BIdmap, contours, cv::Scalar(32, 32, 32));
        }
    }
}

void BImap::FindConcave(const BIpolygons &polygons, std::set<BIangle> &ConcaveSet)
{
    for (int list_i = 0; list_i < polygons.size(); list_i++)
    {
        const std::vector<BIpoint> &subpolygons = polygons[list_i];
        int n = subpolygons.size();
        for (int i = 0; i < n; ++i)
        {
            BIpoint prev = subpolygons[(i + n - 1) % n];
            BIpoint curr = subpolygons[i];
            BIpoint next = subpolygons[(i + 1) % n];

            // 计算向量
            BIpoint vec1 = curr - prev;
            BIpoint vec2 = next - curr;

            // 叉乘检测
            if (vec1.cross(vec2) < 0) // 对于逆时针排列的多边形，叉乘为负表示凹顶点
            {
                // 添加到凹角集合中
                ConcaveSet.insert({curr, prev, next, true});
            }
        }
    }
}

void BImap::FindConcave(const BIpolygons &polygons, std::set<BIangle> &ConcaveSet, std::set<BIangle> &AngleSet)
{
    for (int list_i = 0; list_i < polygons.size(); list_i++)
    {
        const std::vector<BIpoint> &subpolygons = polygons[list_i];
        int n = subpolygons.size();
        for (int i = 0; i < n; ++i)
        {
            BIpoint prev = subpolygons[(i + n - 1) % n];
            BIpoint curr = subpolygons[i];
            BIpoint next = subpolygons[(i + 1) % n];

            // 计算向量
            BIpoint vec1 = curr - prev;
            BIpoint vec2 = next - curr;

            // 叉乘检测
            if (vec1.cross(vec2) < 0) // 对于逆时针排列的多边形，叉乘为负表示凹顶点
            {
                // 添加到凹角集合中
                ConcaveSet.insert({curr, prev, next, true});
                AngleSet.insert({curr, prev, next, true});
            }
            else
            {
                AngleSet.insert({curr, prev, next, false});
            }
        }
    }
}

void BImap::ViewablePoint(const BIangle &ConcaveAngle, const BIpolygons &polygons, const std::list<BIline> cutlineList, std::vector<BIpoint> &VPList)
{
    VPList.clear();
    for (const auto &polygon : polygons)
    {
        for (const auto &point : polygon)
        {
            if (ConcaveAngle.O.x == point.x && ConcaveAngle.O.y == point.y)
                continue; // 跳过点O自己
            // 计算点与角两边的向量的叉积
            double cross1 = crossProduct(ConcaveAngle.O, ConcaveAngle.S, point);
            double cross2 = crossProduct(ConcaveAngle.O, point, ConcaveAngle.E);
            // if (cross1 * cross2 <= 0)
            //     continue; // 如果点在角ConcaveAngle之外，跳过
            if ((ConcaveAngle.Concave && cross1 >= 0 && cross2 >= 0) || (!ConcaveAngle.Concave && (cross1 >= 0 || cross2 >= 0)))
                continue; // 如果点在角ConcaveAngle之外，跳过

            BIline line = {ConcaveAngle.O, point}; // 从O到当前点的线段
            bool isVisible = true;

            for (const auto &poly : polygons)
            {
                for (size_t i = 0; i < poly.size(); ++i)
                {
                    BIpoint start = poly[i];
                    BIpoint end = poly[(i + 1) % poly.size()]; // 循环到多边形的第一个点
                    BIline start2end = {start, end};
                    if (doIntersect(line, start2end))
                    {
                        isVisible = false;
                        break; // 如果线段与多边形的一条边相交，停止检查当前点
                    }
                }
                if (!isVisible)
                    break; // 已确定当前点不可见，检查下一个点
            }
            for (const auto &cutline : cutlineList)
            {
                if (doIntersect(line, cutline))
                {
                    isVisible = false;
                    break; // 如果线段与多边形的一条边相交，停止检查当前点
                }
            }

            if (isVisible)
                VPList.push_back(point);
        }
    }
}
BIpoint BImap::WeightViewablePoint(const BIangle &ConcaveAngle, const BIpolygons &polygons, const std::list<BIline> cutlineList)
{
    BIpoint optPoint = {-1, -1};
    double optCost = doubleMax;
    for (const auto &polygon : polygons)
    {
        for (const auto &point : polygon)
        {
            double nowCost = ConcaveAngle.O % point;
            if (nowCost >= optCost)
                continue; // 跳过点无法优化的点
            if (ConcaveAngle.O.x == point.x && ConcaveAngle.O.y == point.y)
                continue; // 跳过点O自己
            // 计算点与角两边的向量的叉积
            double cross1 = crossProduct(ConcaveAngle.O, ConcaveAngle.S, point);
            double cross2 = crossProduct(ConcaveAngle.O, point, ConcaveAngle.E);
            if ((ConcaveAngle.Concave && cross1 >= 0 && cross2 >= 0) || (!ConcaveAngle.Concave && (cross1 >= 0 || cross2 >= 0)))
                continue; // 如果点在角ConcaveAngle之外，跳过

            BIline line = {ConcaveAngle.O, point}; // 从O到当前点的线段
            bool isVisible = true;

            for (const auto &poly : polygons)
            {
                for (size_t i = 0; i < poly.size(); ++i)
                {
                    BIpoint start = poly[i];
                    BIpoint end = poly[(i + 1) % poly.size()]; // 循环到多边形的第一个点
                    BIline start2end = {start, end};
                    if (doIntersect(line, start2end))
                    {
                        isVisible = false;
                        break; // 如果线段与多边形的一条边相交，停止检查当前点
                    }
                }
                if (!isVisible)
                    break; // 已确定当前点不可见，检查下一个点
            }
            if (!isVisible)
                continue; // 已确定当前点不可见，检查下一个点

            for (const auto &cutline : cutlineList)
            {
                if (doIntersect(line, cutline))
                {
                    isVisible = false;
                    break; // 如果线段与现有分割线相交，停止检查当前点
                }
            }

            if (isVisible)
            {
                optPoint = point;
                optCost = nowCost;
            }
        }
    }
    // if (optPoint.x < 0 || optPoint.y < 0)
    // {
    //     cv::Mat img = cv::Mat::zeros(shapeY, shapeX, CV_8UC3);
    //     drawPolygons(polygons, img);
    //     for (const auto &cutline : cutlineList)
    //     {
    //         cv::Point cutPs = cv::Point((int)(cutline.S.x), (int)(cutline.S.y));
    //         cv::Point cutPe = cv::Point((int)(cutline.E.x), (int)(cutline.E.y));
    //         cv::line(img, cutPs, cutPe, cv::Scalar(0, 127, 0), 1.5);
    //     }
    //     cv::circle(img, cv::Point((int)(ConcaveAngle.O.x), (int)(ConcaveAngle.O.y)), 3, cv::Scalar(0, 0, 255), -1);
    //     cv::imshow("img", img);
    //     cv::waitKey(0);
    //     cv::imwrite("testce.png", img);
    //     cv::destroyAllWindows();
    //     printf("?????");
    // }
    return optPoint;
}

cv::Mat showImg;

int32_t findCommonElement(const std::set<int32_t> &set1, const std::set<int32_t> &set2)
{
    // 快速查找两集合中第(唯)一个元素，用于利用两poly确定其相交的cutline
    // 确定哪个集合较小
    const std::set<int> &smallerSet = (set1.size() < set2.size()) ? set1 : set2;
    const std::set<int> &largerSet = (set1.size() < set2.size()) ? set2 : set1;

    // 遍历较小的集合
    for (const int &element : smallerSet)
    {
        // 如果在较大的集合中找到了这个元素，则返回它
        if (largerSet.find(element) != largerSet.end())
            return element; // 找到共同元素
    }

    // 如果没有找到共同元素，返回空
    return -1;
}

// 反路径清除
void BImap::ReversePathClearing(std::vector<int32_t> &polyPath)
{
    int32_t PathLen = polyPath.size();
    if (PathLen <= 1)
        return;

    int32_t index = 0;
    for (int32_t pointer = 1; pointer < PathLen; pointer++)
    {
        // 如果路径重复，则跳过重复
        if (polyPath[index] == polyPath[pointer])
            continue;

        // 如果路径返回，则回溯引索，当polyPath[index]<0时忽略回溯
        if (polyPath[index] >= 0)
        {
            int32_t index_old = index - 1;
            if (index_old >= 0 && polyPath[index_old] == polyPath[pointer])
            {
                index = index_old;
                continue;
            }
        }
        index++;
        polyPath[index] = polyPath[pointer];
    }
    polyPath.resize(index + 1);
}

void BImap::StartCut(const BIpolygons &polygons, BIgraph &graph)
{
    std::set<BIangle> AngleSet;
    std::set<BIangle> ConcaveSet;
    FindConcave(polygons, ConcaveSet, AngleSet);

    std::list<BIline> cutlineList;
    while (ConcaveSet.size())
    {
        BIangle ConcaveNow = *ConcaveSet.begin();
        ConcaveSet.erase(ConcaveSet.begin());
        AngleSet.erase(ConcaveNow);

        BIpoint optVPoint = WeightViewablePoint(ConcaveNow, polygons, cutlineList);

        BIline newCutline = {ConcaveNow.O, optVPoint};
        cutlineList.push_back(newCutline);
        BIangle newConcave = {{}, {}, {}, false};

        // ConcaveNow.O处新生角的处理
        if (crossProduct(ConcaveNow.O, ConcaveNow.S, optVPoint) > 0)
        {
            newConcave = {ConcaveNow.O, ConcaveNow.S, optVPoint, true};
            ConcaveSet.insert(newConcave);
            AngleSet.insert(newConcave);
            AngleSet.insert({ConcaveNow.O, optVPoint, ConcaveNow.E, false});
        }
        else if (crossProduct(ConcaveNow.O, optVPoint, ConcaveNow.E) > 0)
        {
            newConcave = {ConcaveNow.O, optVPoint, ConcaveNow.E, true};
            ConcaveSet.insert(newConcave);
            AngleSet.insert(newConcave);
            AngleSet.insert({ConcaveNow.O, ConcaveNow.S, optVPoint, false});
        }
        else
        {
            AngleSet.insert({ConcaveNow.O, optVPoint, ConcaveNow.E, false});
            AngleSet.insert({ConcaveNow.O, ConcaveNow.S, optVPoint, false});
        }

        // optVPoint处新生角的处理
        BIangle oppAngle;
        auto it = AngleSet.lower_bound({optVPoint, {-doubleMax, -doubleMax}, {-doubleMax, -doubleMax}});
        while (it != AngleSet.end() && it->O == optVPoint)
        {
            const BIangle &tempAngle = *it;
            // 计算点与角两边的向量的叉积, 如果点在角ConcaveAngle之内
            double cross1 = crossProduct(optVPoint, tempAngle.S, ConcaveNow.O);
            double cross2 = crossProduct(optVPoint, ConcaveNow.O, tempAngle.E);
            if ((tempAngle.Concave && (cross1 < 0 || cross2 < 0)) || (!tempAngle.Concave && cross1 < 0 && cross2 < 0))
                oppAngle = tempAngle;
            ++it;
        }
        if (oppAngle.Concave)
        {
            AngleSet.erase(oppAngle);
            ConcaveSet.erase(oppAngle);
            if (crossProduct(optVPoint, oppAngle.S, ConcaveNow.O) > 0)
            {
                newConcave = {optVPoint, oppAngle.S, ConcaveNow.O, true};
                ConcaveSet.insert(newConcave);
                AngleSet.insert(newConcave);
                AngleSet.insert({optVPoint, ConcaveNow.O, oppAngle.E, false});
            }
            else if (crossProduct(optVPoint, ConcaveNow.O, oppAngle.E) > 0)
            {
                newConcave = {optVPoint, ConcaveNow.O, oppAngle.E, true};
                ConcaveSet.insert(newConcave);
                AngleSet.insert(newConcave);
                AngleSet.insert({optVPoint, oppAngle.S, ConcaveNow.O, false});
            }
            else
            {
                AngleSet.insert({optVPoint, ConcaveNow.O, oppAngle.E, false});
                AngleSet.insert({optVPoint, oppAngle.S, ConcaveNow.O, false});
            }
        }
        else
        {
            AngleSet.erase(oppAngle);
            AngleSet.insert({optVPoint, oppAngle.S, ConcaveNow.O, false});
            AngleSet.insert({optVPoint, ConcaveNow.O, oppAngle.E, false});
        }
    }

    // 根据剖分结果形成图结构
    std::vector<BIcutline> &GcutlineList = graph.cutlineList;
    std::vector<BIfreepolygon> &GfreepolygonList = graph.freepolygonList;
    std::vector<BIobspolygon> &obspolygonList = graph.obspolygonList;

    // 构建分割线与凸多边形的连通关系
    std::map<BIline, int32_t> cutlineMap;
    GcutlineList.reserve(cutlineList.size() + 100); // 额外开辟100空间备用
    for (const BIline &tempCutline : cutlineList)   // 初始化GcutlineList
    {
        int32_t index = GcutlineList.size();
        cutlineMap[tempCutline] = index;
        GcutlineList.push_back({index, tempCutline, (tempCutline.S + tempCutline.E) / 2.0, {}, {}});
    }

    GfreepolygonList.reserve(2 * cutlineList.size() + 200); // 至少额外开辟200空间备用
    std::set<BIangle> tempAngleSet = AngleSet;
    std::map<BIangle, int32_t> angle2freepoly; // 用于后续通过angle寻找freepoly的索引，只需要{O, E}为分割线的angle
    while (tempAngleSet.size())
    {
        GfreepolygonList.emplace_back();
        BIfreepolygon &freepolygon = GfreepolygonList.back(); // 创建一个多边形
        freepolygon.index = GfreepolygonList.size() - 1;

        BIangle beginAngle = *tempAngleSet.begin();
        BIangle tempAngle = beginAngle;
        BIpoint tempCore = {0, 0};
        do
        {
            tempAngleSet.erase(tempAngle);
            BIpoint nextOPoint = tempAngle.E;
            BIline tempLine = {tempAngle.O, nextOPoint};
            tempCore = tempCore + tempAngle.O;

            freepolygon.polygon.push_back(tempAngle.O); // 添加一个多边形顶点

            if (cutlineMap.count(tempLine)) // 添加cutline的关系
            {
                int32_t cutlineIndex = cutlineMap[tempLine];
                freepolygon.cutlinelink.insert(cutlineIndex);
                GcutlineList[cutlineIndex].polygonlink.insert(freepolygon.index);
                angle2freepoly[tempAngle] = freepolygon.index;
            }

            // 查找下一个角
            // 以tempAngle.O -> nextOPoint为起始边S -> 0的角
            auto nextangleIt = AngleSet.lower_bound({nextOPoint, tempAngle.O, {-doubleMax, -doubleMax}});
            if (nextangleIt == AngleSet.end())
            {
                printf("Have Error\r\n");
                return;
            }

            tempAngle = *nextangleIt;
        } while (tempAngle.O != beginAngle.O);
        freepolygon.core = tempCore / freepolygon.polygon.size();
    }
    graph.cutlineBaseNum = GcutlineList.size();
    graph.freepolygonBaseNum = GfreepolygonList.size();

    // 通过BIcutline.polygonlink与BIfreepolygon.cutlinelink建立BIcutline.cutlinelink关系
    for (BIcutline &cutline : GcutlineList)
    {
        std::set<int32_t> &polygonlink = cutline.polygonlink;
        std::set<int32_t> &cutlinelink = cutline.cutlinelink;
        for (const int32_t polygonIndex : polygonlink)
        {
            std::set<int32_t> &polygonCutlink = GfreepolygonList[polygonIndex].cutlinelink;
            cutlinelink.insert(polygonCutlink.begin(), polygonCutlink.end());
        }
        cutlinelink.erase(cutline.index);
    }

    // 通过BIfreepolygon.cutlinelink与BIcutline.polygonlink建立BIfreepolygon.polygonlink关系
    for (BIfreepolygon &freepolygon : GfreepolygonList)
    {
        std::set<int32_t> &polygonlink = freepolygon.polygonlink;
        std::set<int32_t> &cutlinelink = freepolygon.cutlinelink;
        for (const int32_t cutlineIndex : cutlinelink)
        {
            std::set<int32_t> &cutlinePolygonlink = GcutlineList[cutlineIndex].polygonlink;
            polygonlink.insert(cutlinePolygonlink.begin(), cutlinePolygonlink.end());
        }
        polygonlink.erase(freepolygon.index);
    }

    // 构建BIinvnode到cutline的引索关系
    BIinvnodeMap &invnode2cutlineMap = graph.invnode2cutlineMap;
    for (BIcutline &cutline : GcutlineList)
    {
        std::set<int32_t> &polygonlink = cutline.polygonlink;
        int32_t polygonIndex0 = *polygonlink.begin();
        int32_t polygonIndex1 = *polygonlink.rbegin();
        invnode2cutlineMap[{polygonIndex0, polygonIndex1}] = cutline.index;
        invnode2cutlineMap[{polygonIndex1, polygonIndex0}] = cutline.index;
    }

    // 构建障碍物的邻域连通关系
    obspolygonList.resize(polygons.size() - 1); // 开辟障碍物所需空间
    for (int32_t obs_i = 0; obs_i < polygons.size() - 1; obs_i++)
    {
        const BIpolygon &polygonObs = polygons[obs_i + 1];
        BIobspolygon &obspolygon = obspolygonList[obs_i];

        obspolygon.index = obs_i;
        obspolygon.polygon = polygonObs;

        for (size_t i = 0; i < polygonObs.size(); ++i) // 遍历polygonObs的有向边
        {
            BIpoint start = polygonObs[i];
            BIpoint end = polygonObs[(i + 1) % polygonObs.size()]; // 循环到多边形的第一个点

            auto nextangleIt = AngleSet.lower_bound({end, start, {-doubleMax, -doubleMax}});
            if (nextangleIt == AngleSet.end())
            {
                printf("Have Error\r\n");
                return;
            }
            for (;;)
            {
                const BIline angleOE = {nextangleIt->O, nextangleIt->E};
                if (cutlineMap.count(angleOE))
                {
                    int32_t ployIndex = angle2freepoly[*nextangleIt];
                    obspolygon.polygonRmap[ployIndex] = obspolygon.polygonRlist.size();
                    obspolygon.polygonRlist.push_back(ployIndex);
                }
                else
                    break;
                nextangleIt = AngleSet.lower_bound({nextangleIt->O, nextangleIt->E, {-doubleMax, -doubleMax}});
            }
        }
    }
}
