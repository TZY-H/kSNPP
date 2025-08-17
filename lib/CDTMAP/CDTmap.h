#ifndef __CHIsso_H
#define __CHIsso_H
#include <iostream>
#include <stdio.h>
#include <vector>
#include <queue>
#include <list>
#include <set>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <thread>
#include <random>
#include <functional>
// #include <zip.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// #include "CSubdivision.h"
#include <nlohmann/json.hpp>
using json = nlohmann::json;
class BIbridge;
class BIisland;

struct BIpoint
{
    double x;
    double y;
    bool operator==(const BIpoint &other) const
    {
        return std::abs(x - other.x) <= 1e-16 &&
               std::abs(y - other.y) <= 1e-16;
    }
    bool operator!=(const BIpoint &other) const
    {
        return std::abs(x - other.x) > 1e-16 ||
               std::abs(y - other.y) > 1e-16;
    }
    bool operator<(const BIpoint &other) const
    {
        // 在这里定义比较规则
        if (x != other.x)
        {
            return x < other.x;
        }
        return y < other.y;
    }
    BIpoint operator+(const BIpoint &other) const
    {
        return {x + other.x, y + other.y};
    }
    BIpoint operator-(const BIpoint &other) const
    {
        return {x - other.x, y - other.y};
    }
    BIpoint operator*(const double &ratio) const
    {
        return {ratio * x, ratio * y};
    }
    BIpoint operator/(const double &ratio) const
    {
        return {x / ratio, y / ratio};
    }
    double operator%(const BIpoint &other) const
    {
        double dx = x - other.x;
        double dy = y - other.y;
        return sqrt(dx * dx + dy * dy);
    }
    // 叉乘
    double cross(const BIpoint &other) const
    {
        return x * other.y - y * other.x;
    }
    // 从json到BIpoint的转换函数
    void from_json(const json &j)
    {
        x = j.at(0).get<double>();
        y = j.at(1).get<double>();
    }
};

typedef struct
{
    double cost;
    BIpoint point;
    int32_t par; // 父节点
    // int32_t sub;    //子节点
    int32_t bridge; // 所属分割线
    int32_t index;
} Node;

struct BIangle
{
    BIpoint O, S, E;
    bool Concave;
    bool operator<(const BIangle &other) const
    {
        if (O < other.O)
            return true;
        if (O != other.O)
            return false;
        if (S < other.S)
            return true;
        if (S != other.S)
            return false;
        return E < other.E;
    }
    bool operator==(const BIangle &other) const
    {
        return (O == other.O) && (S == other.S) && (E == other.E);
    }
};
typedef std::vector<BIpoint> BIpolygon;
typedef std::vector<BIpolygon> BIpolygons;

struct BIline
{
    BIpoint S;
    BIpoint E;
    // 构造函数，自动排序端点
    BIline(BIpoint p1, BIpoint p2)
    {
        if (p2 < p1)
        {
            S = p2;
            E = p1;
        }
        else
        {
            S = p1;
            E = p2;
        }
    }
    bool operator<(const BIline &other) const
    {
        if (S < other.S)
            return true;
        if (S != other.S)
            return false;
        return E < other.E;
    }
    bool operator==(const BIline &other) const
    {
        return (S == other.S) && (E == other.E);
    }
};

struct BIcutline
{
    int32_t index;
    BIline line;
    BIpoint core;
    std::set<int32_t> polygonlink; // polygon index set
    std::set<int32_t> cutlinelink; // cutline index set
};

struct BIobspolygon
{
    int32_t index;
    BIpolygon polygon;
    std::map<int32_t, int32_t> polygonRmap; // 周边的多边形索引集合
    std::vector<int32_t> polygonRlist;      // 周边的多边形索引有序列表
};

struct BIfreepolygon
{
    int32_t index;
    BIpolygon polygon;
    BIpoint core;
    std::set<int32_t> polygonlink; // polygon index set
    std::set<int32_t> cutlinelink; // cutline index set
};

// struct BIgoalpoint
// {
//     int32_t index;
//     // BIpolygon polygon;
//     BIpoint core;
//     int32_t polygonlink; // polygon index set
// };

struct BIinvnode
{
    int32_t polyS; // 源多边形引索，BIfreepolygon
    int32_t polyE; // 汇多边形引索，BIfreepolygon

    struct Hash {
        std::size_t operator()(const BIinvnode& node) const {
            return std::hash<int32_t>{}(node.polyS) ^ 
                   (std::hash<int32_t>{}(node.polyE) << 1);
        }
    };
    
    // 为了完整性，也可以定义Equal
    struct Equal {
        bool operator()(const BIinvnode& lhs, const BIinvnode& rhs) const {
            return lhs.polyS == rhs.polyS && lhs.polyE == rhs.polyE;
        }
    };

    bool operator<(const BIinvnode &other) const
    {
        if (polyS < other.polyS)
            return true;
        if (polyS != other.polyS)
            return false;
        return polyE < other.polyE;
    }
    bool operator==(const BIinvnode &other) const
    {
        return (polyS == other.polyS) && (polyE == other.polyE);
    }
};
using BIinvnodeMap = std::unordered_map<BIinvnode, int32_t, BIinvnode::Hash, BIinvnode::Equal>;
// using BIinvnodeMap = std::map<BIinvnode, int32_t>;

struct BIgraph
{
    int32_t cutlineBaseNum;
    int32_t freepolygonBaseNum;
    std::vector<BIcutline> cutlineList;
    std::vector<BIobspolygon> obspolygonList;
    std::vector<BIfreepolygon> freepolygonList;
    BIinvnodeMap invnode2cutlineMap;
};

struct THPPtask
{
    BIpoint Origin;          // 起始点
    int32_t OriginPolyIndex; // 起始poly引索
    int32_t graphIndex;      // 图引索
    double TetherLength = 0;

    int8_t mod = 0; // 0:获取系留机器人全可行同伦类、1：系留机器人无冗余环路可行同伦类

    std::vector<int32_t> EncodingSet;        // PolyIndex, HomotopyID (The number of HomotopyPolyIndex)
                                             // HomotopyPolyIndex: HomotopyID<<16 | PolyIndex
    std::map<int32_t, int32_t> EncodingTree; // HomotopyPolyIndex, parentHomotopyPolyIndex

    int32_t HomotopyPolyIndex_Init;
    BIpoint Init;
    BIpoint Goal;
    std::list<BIpoint> minPath;
};

struct kSNPPtask
{
    BIpoint x_init;
    BIpoint x_goal;
    int32_t graphIndex;
    int32_t xx_init;
    int32_t xx_goal;
    int32_t k = 1;
    double MaxCost = 1.79769e+307;
    std::vector<std::list<BIpoint>> Q_kSNP;
    std::vector<double> Q_Costs;
};

#define BIdebug 1
class BImap
{
private:
public:
    uint32_t shapeX;  // 地图x轴像素数
    uint32_t shapeY;  // 地图y轴像素数
    double mapratio;  // 地图分辨率
    double robotsize; // 机器人直径

    cv::Mat BIamap; // 岛群图像
    cv::Mat BIimap; // 岛号图像
    cv::Mat BIdmap; // debug图象
    cv::Mat BIfmap; // free图象
    int testcount = 0;

    void MaptoBInavi(char *IMGmap, double ratio, double rsize, const char *addr = "127.0.0.1", int port = 23231);
    void DrawBImap(bool debugmap);

    std::vector<BIgraph> BIgraphList;
    void FindConcave(const BIpolygons &polygons, std::set<BIangle> &ConcaveSet);
    void FindConcave(const BIpolygons &polygons, std::set<BIangle> &ConcaveSet, std::set<BIangle> &AngleSet);
    void ViewablePoint(const BIangle &ConcaveAngle, const BIpolygons &polygons, const std::list<BIline> cutlineList, std::vector<BIpoint> &VPList);
    BIpoint WeightViewablePoint(const BIangle &ConcaveAngle, const BIpolygons &polygons, const std::list<BIline> cutlineList);
    void StartCut(const BIpolygons &polygons, BIgraph &graph);
    void ReversePathClearing(std::vector<int32_t> &polyPath);
    void GetLeastHomotopyPath(const std::vector<BIline> &f_path, std::list<BIpoint> &Path, double &PathMinCost);

    //**********THPP**********//
    void THPPtaskInit(THPPtask &task, BIpoint &origin, double tl, int8_t mod = 0);                                                  // 初始化THPP任务，mod = 0 系留配置约束，mod = 1 一般最优路径约束
    bool THPPExpandingClassValidity(THPPtask &task, int32_t HomotopyPolyIndexPar, int32_t PolyIndexSub, double threshold = 3);      // 边的有效性检测
    void THPPgetCDTencoding(THPPtask &task, int32_t HomotopyPolyIndex, std::vector<int32_t> &polyPath);                             // 获取CDT编码，回溯HomotopyPoly编码树
    void THPPgetCDTencodingCutline(THPPtask &task, int32_t HomotopyPolyIndex, std::vector<BIline> &cpath, BIpoint goal = {-1, -1}); // 获取CDT编码的对偶形式

    double THPPoptimalReConfig(THPPtask &task, std::vector<int32_t> &polyPathS, std::vector<int32_t> &polyPathG,
                               BIpoint Init, BIpoint Goal, std::list<BIpoint> &minPath); // 待测试
    double THPPoptimalPlanner(THPPtask &task, int32_t HomotopyPolyIndex_Init, int32_t &HomotopyPolyIndex_Goal,
                              BIpoint Init, BIpoint Goal, std::list<BIpoint> &minPath);                  // 系留机器人最优路径规划器，完整形式，初始化mod0
    double THPPoptimalPlanner(THPPtask &task);                                                           // 系留机器人最优路径规划器，简化形式
    double UTHPPoptimalPlanner(THPPtask &task, BIpoint Init, BIpoint Goal, std::list<BIpoint> &minPath); // 非系留机器人最优路径规划器，初始化mod1
                                                                                                         // 以初步测试
    double TMVoptimalPlanner(THPPtask &task, int32_t HomotopyPolyIndex_Init, BIpoint Init,
                             std::vector<BIpoint> Goals, std::list<BIpoint> &minPath); // 系留机器人最优多目标访问规划器，初始化mod0
    double TMVoptimalPlannerViolent(THPPtask &task, int32_t HomotopyPolyIndex_Init, BIpoint Init,
                                    std::vector<BIpoint> Goals, std::list<BIpoint> &minPath); // 暴力搜索版本，对比算法

    void GetAllOptConfigurations(THPPtask &task, BIpoint goal,
                                 std::map<int32_t, std::pair<std::list<BIpoint>, double>>
                                     &ConfigList);

    //**********kSNPP**********//
    int64_t test_count = 0;
    // int64_t test_count = 0;
    void kSNPPlanner(kSNPPtask &task);
    void kSNPPlanner(kSNPPtask &task, BIpoint x_init, BIpoint x_goal, int32_t k, bool plusKey = true);
    void kSNPPlannerPlus(kSNPPtask &task); //推荐使用Plus，通过复用临时树，进一步加速GetLeastHomPath过程
    void ReduceBranches(int32_t graphIndex, int32_t xx_init, int32_t xx_goal, std::set<int32_t> &InvPolygons);
    double GetLeastHomPathPlus(BIpoint &point,
                               int32_t HomPolyIndex,
                               int32_t &HomPointIndexPar,
                               const int32_t InitPolyIndex,
                               const BIpoint &x_Init,
                               const std::vector<BIcutline> &cutlineList,
                               BIinvnodeMap &invnode2cutlineMap,
                               std::unordered_map<int32_t, int32_t> &EncodingTree,
                               std::unordered_map<int32_t, std::pair<double, int32_t>> &EPointTree);
    void kSNP2THCSearcher(kSNPPtask &task, BIpoint x_init, BIpoint x_goal, double TetherLength, int32_t k);


    BImap();
    ~BImap();
};

// 通用函数与参数 >>>
#define doubleMax (1.79769e+307)
// vector<pair<int32_t, size_t>>的哈希计算器
size_t hash_vec64(const std::vector<std::pair<int32_t, size_t>> &vec);
// 返回第一个共同元素，已弃用
int32_t findCommonElement(const std::set<int32_t> &set1, const std::set<int32_t> &set2);
// 简化的计时器
int64_t utime_ns(void);
// 计算点p到点o连线沿x轴正方向顺时针的夹角（单位：弧度）
double calculateAngle(BIpoint o, BIpoint p);
// 计算叉乘
double crossProduct(const BIpoint &O, const BIpoint &A, const BIpoint &B);
// 松弛的：检查两条线段是否相交，无视端点，线段长度不为0
bool doIntersect(const BIline &l1, const BIline &l2);
// 严谨的：检查两条线段是否相交，检查端点，长度可为0
bool doIntersect_rigorous(const BIline &l1, const BIline &l2);
// 计算点到线段的距离
double Point2LineDistance(BIpoint p, BIpoint l_s, BIpoint l_e);

// 可视化调试函数 >>>
void drawBIgraphObs(BIgraph &graph, cv::Mat image);
void drawPolygons(const BIpolygons &polygons, cv::Mat image);
void drawConcaves(std::list<BIangle> &ConcaveSet, cv::Mat image);
void drawAngle(BIangle &nowAngle, cv::Mat image);
void drawBIgraph(BIgraph &graph, cv::Mat image);

void drawBIring(std::list<BIpoint> &Path, cv::Mat &image);
void drawBIpath(const std::list<BIpoint> &path, cv::Mat &image, int32_t waittime = 30, cv::Scalar color = cv::Scalar(255, 0, 0));
void drawBIfreeID(const BIgraph &graph, cv::Mat &image);

#endif