#include "CDTmap.h"
#include <math.h>

void BImap::kSNPPlanner(kSNPPtask &task, BIpoint x_init, BIpoint x_goal, int32_t k, bool plusKey)
{
    task.x_init = x_init;
    task.x_goal = x_goal;
    task.k = k;
    if (plusKey)
        kSNPPlannerPlus(task);
    else
        kSNPPlanner(task);
}

void BImap::kSNP2THCSearcher(kSNPPtask &task, BIpoint x_init, BIpoint x_goal, double TetherLength, int32_t k)
{
    task.x_init = x_init;
    task.x_goal = x_goal;
    task.MaxCost = TetherLength;
    task.k = k;
    kSNPPlannerPlus(task);
}

void BImap::ReduceBranches(int32_t graphIndex, int32_t xx_init, int32_t xx_goal, std::set<int32_t> &InvPolygons)
{
    std::queue<int32_t> V_sin;
    InvPolygons.clear();
    std::vector<BIfreepolygon> &freepolygonList = BIgraphList[graphIndex].freepolygonList;
    std::map<int32_t, int32_t> PolygonReduceNum;

    for (int32_t index = 0; index < freepolygonList.size(); index++)
    {
        if (freepolygonList[index].polygonlink.size() == 1 &&
            index != xx_init && index != xx_goal)
        {
            V_sin.push(index);
        }
    }

    while (V_sin.size())
    {
        int32_t PolygonNow = V_sin.front();
        V_sin.pop();
        InvPolygons.insert(PolygonNow);
        std::set<int32_t> &polygonlink = freepolygonList[PolygonNow].polygonlink;
        // for (int32_t index = 0; index < polygonlink.size(); index++)
        int32_t validLink = 0;
        for (int32_t polygonNear : polygonlink)
        {
            if (!InvPolygons.count(polygonNear))
            {
                validLink = polygonNear;
                break;
            }
        }
        if (validLink != xx_init && validLink != xx_goal)
        {
            PolygonReduceNum[validLink]++;
            if (freepolygonList[validLink].polygonlink.size() -
                    PolygonReduceNum[validLink] ==
                1)
            {
                V_sin.push(validLink);
            }
        }
    }
}

void BImap::kSNPPlanner(kSNPPtask &task)
{
    task.Q_Costs.clear();
    task.Q_kSNP.clear();

    BIpoint &x_init = task.x_init;
    BIpoint &x_goal = task.x_goal;

    uint32_t anum_Init = BIamap.at<uint16_t>(x_init.y, x_init.x);
    uint32_t anum_Goal = BIamap.at<uint16_t>(x_goal.y, x_goal.x);
    if (anum_Init == 0xffff || anum_Init != anum_Goal)
    {
        printf("error: robot can't be here!!\r\n");
        return;
    }

    int32_t PolyIndex_Init = BIimap.at<uint16_t>(x_init.y, x_init.x);
    int32_t PolyIndex_Goal = BIimap.at<uint16_t>(x_goal.y, x_goal.x);
    task.graphIndex = anum_Init;
    task.xx_init = PolyIndex_Init;
    task.xx_goal = PolyIndex_Goal;

    BIgraph &graph = BIgraphList[anum_Init];
    std::vector<BIcutline> &cutlineList = graph.cutlineList;
    std::vector<BIfreepolygon> &freepolygonList = graph.freepolygonList;
    BIinvnodeMap &invnode2cutlineMap = graph.invnode2cutlineMap;

    std::set<int32_t> InvPolygons;
    ReduceBranches(anum_Init, PolyIndex_Init, PolyIndex_Goal, InvPolygons);

    // 添加虚拟多边形
    int32_t VirtualPolygon = 0x0000FFFF;
    int32_t VirtualEncodingSet = -1;

    std::vector<int32_t> EncodingSet;
    std::map<int32_t, int32_t> EncodingTree;
    EncodingSet.resize(freepolygonList.size(), -1);
    std::map<int32_t, double> EncodingGcost;
    std::map<int32_t, std::list<BIpoint>> SNPPmap;

    std::priority_queue<std::pair<double, int32_t>,
                        std::vector<std::pair<double, int32_t>>,
                        std::greater<std::pair<double, int32_t>>>
        Q_var;
    std::vector<int32_t> Q_kSNPvar;

    EncodingSet[PolyIndex_Init]++;
    int32_t PathEncoding_Init = PolyIndex_Init;
    EncodingTree[PathEncoding_Init] = -1;
    EncodingGcost[PathEncoding_Init] = 0;
    Q_var.push({x_init % x_goal, PathEncoding_Init});

    while (Q_kSNPvar.size() < task.k && Q_var.size())
    {
        double CostNow = Q_var.top().first;
        int32_t PathEncodingNow = Q_var.top().second;
        int32_t PolygonNow = PathEncodingNow & 0x0000FFFF;
        Q_var.pop();
        if (task.MaxCost < CostNow)
            break;

        if (PolygonNow == VirtualPolygon)
        {
            Q_kSNPvar.push_back(PathEncodingNow);
            task.Q_Costs.push_back(CostNow);
            task.Q_kSNP.emplace_back();
            std::swap(task.Q_kSNP.back(), SNPPmap[PathEncodingNow]);
            continue;
        }

        int32_t PathEncodingPar = EncodingTree[PathEncodingNow];
        int32_t PolyIndexPar = PathEncodingPar & 0x0000FFFF;
        for (int32_t PnearIndex : freepolygonList[PolygonNow].polygonlink)
        {
            if (PnearIndex == PolyIndexPar || InvPolygons.count(PnearIndex))
                continue;

            EncodingSet[PnearIndex]++;
            int32_t PathEncodingSub = (EncodingSet[PnearIndex] << 16) | PnearIndex;

            int32_t cutline_index = invnode2cutlineMap[{PnearIndex, PolygonNow}];
            BIcutline &cutline = graph.cutlineList[cutline_index];
            // 计算fCost
            double hCost = Point2LineDistance(x_goal, cutline.line.S, cutline.line.E);

            std::vector<BIline> cpath;
            cpath.emplace_back(cutline.core, cutline.core);
            int32_t PathHomotopyPolyIndex_Now = PathEncodingNow;
            int32_t PathHomotopyPolyIndex_Par = PathEncodingPar;
            while (PathHomotopyPolyIndex_Par != -1)
            {
                int32_t PathPolyIndexPar = PathHomotopyPolyIndex_Par & 0x0000FFFF;
                int32_t PathPolyIndexNow = PathHomotopyPolyIndex_Now & 0x0000FFFF;
                int32_t cutlineIndex_Par2Now = invnode2cutlineMap[{PathPolyIndexPar, PathPolyIndexNow}];
                cpath.push_back(cutlineList[cutlineIndex_Par2Now].line);
                PathHomotopyPolyIndex_Now = PathHomotopyPolyIndex_Par;
                PathHomotopyPolyIndex_Par = EncodingTree[PathHomotopyPolyIndex_Now];
            }
            cpath.emplace_back(x_init, x_init);
            double CosttempM;
            std::list<BIpoint> Pathtemp;
            GetLeastHomotopyPath(cpath, Pathtemp, CosttempM);

            double gCost = std::max(CosttempM - (cutline.line.S % cutline.core),
                                    EncodingGcost[PathEncodingNow]);

            double fCost = gCost + hCost;
            Q_var.push({fCost, PathEncodingSub});

            EncodingTree[PathEncodingSub] = PathEncodingNow;
            EncodingGcost[PathEncodingSub] = gCost;
        }

        if (PolygonNow == PolyIndex_Goal)
        {
            std::vector<BIline> cpath;
            cpath.emplace_back(x_goal, x_goal);
            int32_t PathHomotopyPolyIndex_Now = PathEncodingNow;
            int32_t PathHomotopyPolyIndex_Par = PathEncodingPar;
            while (PathHomotopyPolyIndex_Par != -1)
            {
                int32_t PathPolyIndexPar = PathHomotopyPolyIndex_Par & 0x0000FFFF;
                int32_t PathPolyIndexNow = PathHomotopyPolyIndex_Now & 0x0000FFFF;
                int32_t cutlineIndex_Par2Now = invnode2cutlineMap[{PathPolyIndexPar, PathPolyIndexNow}];
                cpath.push_back(cutlineList[cutlineIndex_Par2Now].line);
                PathHomotopyPolyIndex_Now = PathHomotopyPolyIndex_Par;
                PathHomotopyPolyIndex_Par = EncodingTree[PathHomotopyPolyIndex_Now];
            }
            cpath.emplace_back(x_init, x_init);
            double Costtemp;
            std::list<BIpoint> Pathtemp;
            GetLeastHomotopyPath(cpath, Pathtemp, Costtemp);

            VirtualEncodingSet++;
            int32_t VirtualPathEncoding = (VirtualEncodingSet << 16) | VirtualPolygon;
            std::swap(SNPPmap[VirtualPathEncoding], Pathtemp);
            Q_var.push({Costtemp, VirtualPathEncoding});

            EncodingTree[VirtualPathEncoding] = PathEncodingNow;
        }
    }
}

void BImap::kSNPPlannerPlus(kSNPPtask &task)
{
    task.Q_Costs.clear();
    task.Q_kSNP.clear();

    BIpoint &x_init = task.x_init;
    BIpoint &x_goal = task.x_goal;

    uint32_t anum_Init = BIamap.at<uint16_t>(x_init.y, x_init.x);
    uint32_t anum_Goal = BIamap.at<uint16_t>(x_goal.y, x_goal.x);
    if (anum_Init == 0xffff || anum_Init != anum_Goal)
    {
        printf("error: robot can't be here!!\r\n");
        return;
    }

    int32_t PolyIndex_Init = BIimap.at<uint16_t>(x_init.y, x_init.x);
    int32_t PolyIndex_Goal = BIimap.at<uint16_t>(x_goal.y, x_goal.x);
    task.graphIndex = anum_Init;
    task.xx_init = PolyIndex_Init;
    task.xx_goal = PolyIndex_Goal;

    BIgraph &graph = BIgraphList[anum_Init];
    std::vector<BIcutline> &cutlineList = graph.cutlineList;
    std::vector<BIfreepolygon> &freepolygonList = graph.freepolygonList;
    BIinvnodeMap &invnode2cutlineMap = graph.invnode2cutlineMap;

    std::set<int32_t> InvPolygons;
    ReduceBranches(anum_Init, PolyIndex_Init, PolyIndex_Goal, InvPolygons);

    // 添加虚拟多边形
    int32_t VirtualPolygon = 0x0000FFFF;
    int32_t VirtualEncodingSet = -1;

    std::vector<int32_t> EncodingSet;
    std::unordered_map<int32_t, int32_t> EncodingTree;
    std::unordered_map<int32_t, std::pair<double, int32_t>> EPointTree; // (SEsig<<31) | pathEncoding : (cost, par)

    EncodingSet.resize(freepolygonList.size(), -1);
    std::unordered_map<int32_t, double> EncodingGcost;
    std::map<int32_t, std::list<BIpoint>> SNPPmap;

    std::priority_queue<std::pair<double, int32_t>,
                        std::vector<std::pair<double, int32_t>>,
                        std::greater<std::pair<double, int32_t>>>
        Q_var;
    std::vector<int32_t> Q_kSNPvar;

    EncodingSet[PolyIndex_Init]++;
    int32_t PathEncoding_Init = PolyIndex_Init;
    EncodingTree[PathEncoding_Init] = -1;
    EncodingGcost[PathEncoding_Init] = 0;
    EPointTree[PathEncoding_Init] = {0, -1};

    Q_var.push({x_init % x_goal, PathEncoding_Init});

    while (Q_kSNPvar.size() < task.k && Q_var.size())
    {
        double CostNow = Q_var.top().first;
        int32_t PathEncodingNow = Q_var.top().second;
        int32_t PolygonNow = PathEncodingNow & 0x0000FFFF;
        Q_var.pop();
        if (task.MaxCost < CostNow)
            break;

        if (PolygonNow == VirtualPolygon)
        {
            Q_kSNPvar.push_back(PathEncodingNow);
            task.Q_Costs.push_back(CostNow);
            task.Q_kSNP.emplace_back();
            std::swap(task.Q_kSNP.back(), SNPPmap[PathEncodingNow]);
            continue;
        }

        int32_t PathEncodingPar = EncodingTree[PathEncodingNow];
        int32_t PolyIndexPar = PathEncodingPar & 0x0000FFFF;
        for (int32_t PnearIndex : freepolygonList[PolygonNow].polygonlink)
        {
            if (PnearIndex == PolyIndexPar || InvPolygons.count(PnearIndex))
                continue;

            EncodingSet[PnearIndex]++;
            int32_t PathEncodingSub = (EncodingSet[PnearIndex] << 16) | PnearIndex;

            int32_t cutline_index = invnode2cutlineMap[{PnearIndex, PolygonNow}];
            BIcutline &cutline = graph.cutlineList[cutline_index];
            // 计算fCost
            double hCost = Point2LineDistance(x_goal, cutline.line.S, cutline.line.E);

            // 带扩展的PathEncoding还未处理，由于cutline在PathEncodingNow与PathEncodingSub之间
            // 因此对于cutline上最优同伦路径的计算依然可使用PathEncodingNow
            int32_t EPointIndexPar;
            double CosttempM = GetLeastHomPathPlus(cutline.core, PathEncodingNow, EPointIndexPar,
                                                   PathEncoding_Init, x_init, cutlineList,
                                                   invnode2cutlineMap, EncodingTree, EPointTree);

            double gCost = std::max(CosttempM - (cutline.line.S % cutline.core),
                                    EncodingGcost[PathEncodingNow]);

            double fCost = gCost + hCost;
            Q_var.push({fCost, PathEncodingSub});

            // 扩展cutline的端点进入EPointTree
            double CosttempS = GetLeastHomPathPlus(cutline.line.S, PathEncodingNow, EPointIndexPar,
                                                   PathEncoding_Init, x_init, cutlineList,
                                                   invnode2cutlineMap, EncodingTree, EPointTree);
            EPointTree[PathEncodingSub] = {CosttempS, EPointIndexPar};

            double CosttempE = GetLeastHomPathPlus(cutline.line.E, PathEncodingNow, EPointIndexPar,
                                                   PathEncoding_Init, x_init, cutlineList,
                                                   invnode2cutlineMap, EncodingTree, EPointTree);
            EPointTree[PathEncodingSub | 0x80000000] = {CosttempE, EPointIndexPar};

            EncodingTree[PathEncodingSub] = PathEncodingNow;
            EncodingGcost[PathEncodingSub] = gCost;
        }

        if (PolygonNow == PolyIndex_Goal)
        {
            int32_t EPointIndexPar;
            double Costtemp = GetLeastHomPathPlus(x_goal, PathEncodingNow, EPointIndexPar,
                                                  PathEncoding_Init, x_init, cutlineList,
                                                  invnode2cutlineMap, EncodingTree, EPointTree);

            std::list<BIpoint> Pathtemp;
            Pathtemp.push_front(x_goal);
            while (EPointIndexPar != PathEncoding_Init)
            {
                int32_t HomPolyIndex_HPIPar = EPointIndexPar & 0x7FFFFFFF;
                int32_t CLindex =
                    invnode2cutlineMap[{HomPolyIndex_HPIPar & 0x0000FFFF,
                                        EncodingTree[HomPolyIndex_HPIPar] & 0x0000FFFF}];
                if (EPointIndexPar & 0x80000000)
                    Pathtemp.push_front(cutlineList[CLindex].line.E);
                else
                    Pathtemp.push_front(cutlineList[CLindex].line.S);

                EPointIndexPar = EPointTree[EPointIndexPar].second;
            }
            Pathtemp.push_front(x_init);

            VirtualEncodingSet++;
            int32_t VirtualPathEncoding = (VirtualEncodingSet << 16) | VirtualPolygon;
            std::swap(SNPPmap[VirtualPathEncoding], Pathtemp);
            Q_var.push({Costtemp, VirtualPathEncoding});

            EncodingTree[VirtualPathEncoding] = PathEncodingNow;
        }
    }

    // std::cout << "invnode2cutlineMap: " << invnode2cutlineMap.size() << std::endl;
}

double BImap::GetLeastHomPathPlus(BIpoint &point,
                                  int32_t HomPolyIndex,
                                  int32_t &HomPointIndexPar,
                                  const int32_t InitPolyIndex,
                                  const BIpoint &x_Init,
                                  const std::vector<BIcutline> &cutlineList,
                                  BIinvnodeMap &invnode2cutlineMap,
                                  std::unordered_map<int32_t, int32_t> &EncodingTree,
                                  std::unordered_map<int32_t, std::pair<double, int32_t>> &EPointTree)
{
    // int32_t InitPolyIndex = task.InitPolyIndex;
    if (HomPolyIndex == InitPolyIndex)
    {
        HomPointIndexPar = InitPolyIndex;
        return point % x_Init;
    }

    int32_t CLindexO =
        invnode2cutlineMap[{HomPolyIndex & 0x0000FFFF,
                            EncodingTree[HomPolyIndex] & 0x0000FFFF}];
    const BIcutline &cutlineO = cutlineList[CLindexO];

    double returnCostS, returnCostE;
    BIpoint pointPar = cutlineO.line.S;
    int32_t HPointIndexS = HomPolyIndex; // 存储目前可行的父节点编码
    while (1)
    {
        // 确定回溯父节点与对应坐标点、对应同类编码引索
        int32_t HPointIndexPar = EPointTree[HPointIndexS].second;
        int32_t HomPolyIndex_HPIPar = HPointIndexPar & 0x7FFFFFFF;
        BIpoint pointParNow;
        if (HPointIndexPar == InitPolyIndex)
            pointParNow = x_Init;
        else
        {
            int32_t CLindex =
                invnode2cutlineMap[{HomPolyIndex_HPIPar & 0x0000FFFF,
                                    EncodingTree[HomPolyIndex_HPIPar] & 0x0000FFFF}];
            if (HPointIndexPar & 0x80000000)
                pointParNow = cutlineList[CLindex].line.E;
            else
                pointParNow = cutlineList[CLindex].line.S;
        }

        // 遍历HomPolyIndex至HomPolyIndex_HPIPar间的cutline判断相交性
        BIline PtoPar = {point, pointParNow};
        int32_t HomPolyIndex_temp = HomPolyIndex;
        bool should_break = false;
        while (HomPolyIndex_temp != HomPolyIndex_HPIPar)
        {
            int32_t HomPolyIndex_tempPar = EncodingTree[HomPolyIndex_temp];
            int32_t CLindex =
                invnode2cutlineMap[{HomPolyIndex_temp & 0x0000FFFF,
                                    HomPolyIndex_tempPar & 0x0000FFFF}];
            test_count++;
            if (!doIntersect_rigorous(PtoPar, cutlineList[CLindex].line))
            {
                should_break = true;
                break;
            }
            HomPolyIndex_temp = HomPolyIndex_tempPar;
        }
        if (should_break)
        {
            returnCostS = pointPar % point + EPointTree[HPointIndexS].first;
            break;
        }

        HPointIndexS = HPointIndexPar;
        pointPar = pointParNow;
        if (HPointIndexS == InitPolyIndex) // 此时已成果回溯到了x_Init
        {
            HomPointIndexPar = HPointIndexS;
            return point % x_Init;
        } // 注意在EPointTree中InitPolyIndex=PathEncoding_Init即对应x_init
    }

    int32_t HPointIndexE = HomPolyIndex | 0x80000000; // 存储目前可行的父节点编码
    pointPar = cutlineO.line.E;
    while (1)
    {
        // 确定回溯父节点与对应坐标点、对应同类编码引索
        int32_t HPointIndexPar = EPointTree[HPointIndexE].second;
        int32_t HomPolyIndex_HPIPar = HPointIndexPar & 0x7FFFFFFF;
        BIpoint pointParNow;
        if (HPointIndexPar == InitPolyIndex)
            pointParNow = x_Init;
        else
        {
            int32_t CLindex =
                invnode2cutlineMap[{HomPolyIndex_HPIPar & 0x0000FFFF,
                                    EncodingTree[HomPolyIndex_HPIPar] & 0x0000FFFF}];
            if (HPointIndexPar & 0x80000000)
                pointParNow = cutlineList[CLindex].line.E;
            else
                pointParNow = cutlineList[CLindex].line.S;
        }

        // 遍历HomPolyIndex至HomPolyIndex_HPIPar间的cutline判断相交性
        BIline PtoPar = {point, pointParNow};
        int32_t HomPolyIndex_temp = HomPolyIndex;
        bool should_break = false;
        while (HomPolyIndex_temp != HomPolyIndex_HPIPar)
        {
            int32_t HomPolyIndex_tempPar = EncodingTree[HomPolyIndex_temp];
            int32_t CLindex =
                invnode2cutlineMap[{HomPolyIndex_temp & 0x0000FFFF,
                                    HomPolyIndex_tempPar & 0x0000FFFF}];
            test_count++;
            if (!doIntersect_rigorous(PtoPar, cutlineList[CLindex].line))
            {
                should_break = true;
                break;
            }
            HomPolyIndex_temp = HomPolyIndex_tempPar;
        }
        if (should_break)
        {
            returnCostE = pointPar % point + EPointTree[HPointIndexE].first;
            break;
        }

        HPointIndexE = HPointIndexPar;
        pointPar = pointParNow;
        if (HPointIndexE == InitPolyIndex)
        {
            HomPointIndexPar = HPointIndexE;
            return point % x_Init;
        }
    }

    if (returnCostS <= returnCostE)
    {
        HomPointIndexPar = HPointIndexS;
        return returnCostS;
    }
    HomPointIndexPar = HPointIndexE;
    return returnCostE;
}