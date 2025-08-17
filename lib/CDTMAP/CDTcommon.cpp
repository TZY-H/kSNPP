#include "CDTmap.h"

int64_t utime_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<int64_t>(1000000000UL) * static_cast<long long>(ts.tv_sec) +
           static_cast<int64_t>(ts.tv_nsec);
}

BImap::BImap()
{
}
BImap::~BImap()
{
}

void BImap::GetLeastHomotopyPath(const std::vector<BIline> &f_path, std::list<BIpoint> &Path, double &PathMinCost)
{
    int32_t fszie = f_path.size();
    std::vector<Node> nodeList;
    nodeList.reserve(fszie * 2);
    nodeList.emplace_back(Node{0, f_path[0].S, -1, 0, 0});
    nodeList.emplace_back(Node{0, f_path[0].E, -1, 0, 0});
    for (int32_t line_i = 1; line_i < fszie; line_i++)
    {
        int32_t now_0 = nodeList.size();
        nodeList.emplace_back(Node{doubleMax, f_path[line_i].S, -1, line_i, now_0});
        nodeList.emplace_back(Node{doubleMax, f_path[line_i].E, -1, line_i, now_0 + 1});
        for (int32_t nowBias = 0; nowBias < 2; nowBias++)
        {
            Node &x_now = nodeList[now_0 + nowBias];
            for (int32_t p0 = now_0 - 2; p0 < now_0; p0++)
            {
                Node *x_p = &nodeList[p0];
                bool should_break = false;
                while (x_p->index != 0)
                {
                    int32_t p_temp = x_p->par;
                    int32_t line_j = nodeList[p_temp].bridge + 1;
                    BIline l_ptemp2now = {nodeList[p_temp].point, x_now.point};
                    for (int32_t line_k = line_j; line_k < line_i; line_k++)
                    {
                        test_count++;
                        if (!doIntersect_rigorous(l_ptemp2now, f_path[line_k]))
                        {
                            should_break = true;
                            break;
                        }
                    }
                    if (should_break)
                        break;
                    x_p = &nodeList[p_temp];
                }
                double cost_now = x_p->cost + (x_p->point % x_now.point);
                if (x_now.cost > cost_now)
                {
                    x_now.par = x_p->index;
                    x_now.cost = cost_now;
                }
            }
        }
    }
    Path.clear();
    int32_t x = nodeList.back().index;
    while (x != -1)
    {
        Path.push_front(nodeList[x].point);
        x = nodeList[x].par;
    }
    PathMinCost = nodeList.back().cost;
}

// vector<pair<int32_t, size_t>>的哈希计算器
size_t hash_vec64(const std::vector<std::pair<int32_t, size_t>> &vec)
{
    size_t hashValue = 0;
    for (auto &elem : vec)
    {
        // 将每个元素的哈希值与累积的哈希值进行组合
        size_t elemHash = std::hash<int32_t>{}(elem.first) ^ std::hash<size_t>{}(elem.second);
        // 使用一种更均匀的组合方式，比如加法
        hashValue += elemHash;
        // 使用一种更均匀的混合哈希值的方式，比如乘法
        hashValue ^= elemHash + 0x9e3779b9 + (hashValue << 6) + (hashValue >> 2);
    }
    return hashValue;
}

// 计算点p到点o连线沿x轴正方向顺时针的夹角（单位：弧度）
double calculateAngle(BIpoint o, BIpoint p)
{
    // 计算向量op的方向角度（弧度）
    double angle = atan2(p.y - o.y, p.x - o.x);

    // 将弧度转换为角度（0~360度）
    angle = fmod(angle * 180.0 / M_PI + 360.0, 360.0);

    // 调整角度，使其沿x轴正方向顺时针
    angle = 360.0 - angle;

    return angle;
}

// 计算叉乘
double crossProduct(const BIpoint &O, const BIpoint &A, const BIpoint &B)
{
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

// 松弛的：检查两条线段是否相交，无视端点，线段长度不为0
bool doIntersect(const BIline &l1, const BIline &l2)
{
    double cp1 = crossProduct(l1.S, l1.E, l2.S);
    double cp2 = crossProduct(l1.S, l1.E, l2.E);
    double cp3 = crossProduct(l2.S, l2.E, l1.S);
    double cp4 = crossProduct(l2.S, l2.E, l1.E);

    // 如果两个线段位于同一条直线上
    if (cp1 == 0 && cp2 == 0 && cp3 == 0 && cp4 == 0)
    {
        // 检查线段端点是否在对方线段上
        if (std::max(l1.S.x, l1.E.x) <= std::min(l2.S.x, l2.E.x) ||
            std::max(l1.S.y, l1.E.y) <= std::min(l2.S.y, l2.E.y) ||
            std::max(l2.S.x, l2.E.x) <= std::min(l1.S.x, l1.E.x) ||
            std::max(l2.S.y, l2.E.y) <= std::min(l1.S.y, l1.E.y))
            return false;
        else
            return true;
    }

    // 如果两个线段一条线段的端点在另一条线段的两侧
    if ((cp1 * cp2 < 0) && (cp3 * cp4 < 0))
        return true;

    return false;
}

// 严谨的：检查两条线段是否相交，检查端点，长度可为0
bool doIntersect_rigorous(const BIline &l1, const BIline &l2)
{
    if (l1.S == l1.E)
    {
        if (fabs((l2.S % l2.E) - (l2.S % l1.S) - (l1.S % l2.E)) < 1e-32)
            return true;
        else
            return false;
    }
    else if (l2.S == l2.E)
    {
        if (fabs((l1.S % l1.E) - (l1.S % l2.S) - (l2.S % l1.E)) < 1e-32)
            return true;
        else
            return false;
    }
    double cp1 = crossProduct(l1.S, l1.E, l2.S);
    double cp2 = crossProduct(l1.S, l1.E, l2.E);
    double cp3 = crossProduct(l2.S, l2.E, l1.S);
    double cp4 = crossProduct(l2.S, l2.E, l1.E);

    // 如果两个线段位于同一条直线上
    if (cp1 == 0 && cp2 == 0 && cp3 == 0 && cp4 == 0)
    {
        // 检查线段端点是否在对方线段上
        if (std::max(l1.S.x, l1.E.x) < std::min(l2.S.x, l2.E.x) ||
            std::max(l1.S.y, l1.E.y) < std::min(l2.S.y, l2.E.y) ||
            std::max(l2.S.x, l2.E.x) < std::min(l1.S.x, l1.E.x) ||
            std::max(l2.S.y, l2.E.y) < std::min(l1.S.y, l1.E.y))
            return false;
        else
            return true;
    }

    // 如果两个线段一条线段的端点在另一条线段的两侧
    if ((cp1 * cp2 <= 0) && (cp3 * cp4 <= 0))
        return true;

    return false;
}

// 计算点到线段的距离
double Point2LineDistance(BIpoint p, BIpoint l_s, BIpoint l_e)
{
    // 如果线段两端点重合，则距离为点到该点的距离
    if (l_s.x == l_e.x && l_s.y == l_e.y)
    {
        return p % l_s;
    }

    // 计算向量
    double vec_x = l_e.x - l_s.x;
    double vec_y = l_e.y - l_s.y;

    // 计算投影参数t，表示点p在线段延长线上的投影位置
    // t = ((p - l_s) · (l_e - l_s)) / |l_e - l_s|²
    double t = ((p.x - l_s.x) * vec_x + (p.y - l_s.y) * vec_y) / (vec_x * vec_x + vec_y * vec_y);

    // 根据t的值判断最近点的位置
    if (t < 0.0)
    {
        // 投影在线段起点之前，最近点是线段起点
        return p % l_s;
    }
    else if (t > 1.0)
    {
        // 投影在线段终点之后，最近点是线段终点
        return p % l_e;
    }
    else
    {
        // 投影在线段上，计算点到直线的距离
        // 使用叉积公式：|((l_e - l_s) × (p - l_s))| / |l_e - l_s|
        double cross = vec_x * (p.y - l_s.y) - vec_y * (p.x - l_s.x);
        return fabs(cross) / sqrt(vec_x * vec_x + vec_y * vec_y);
    }
}