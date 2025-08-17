#include "CDTmap.h"
#include <math.h>

void BImap::THPPtaskInit(THPPtask &task, BIpoint &Origin, double tl, int8_t mod)
{
    uint32_t anum_O = BIamap.at<uint16_t>(Origin.y, Origin.x);
    if (anum_O == 0xffff)
    {
        printf("error: robot can't be here!!\r\n");
        return;
    }
    cv::Mat &FOPmap = BIimap;
    task.Origin = Origin;
    task.graphIndex = anum_O;
    task.OriginPolyIndex = FOPmap.at<uint16_t>(Origin.y, Origin.x);

    BIgraph &graph = BIgraphList[anum_O];
    std::vector<BIfreepolygon> &freepolygonList = graph.freepolygonList;

    task.Origin = Origin;
    if (tl <= 0)
    {
        printf("error: TetherLength must be greater than 0!!\r\n");
        return;
    }
    task.mod = mod;
    task.TetherLength = tl;
    std::vector<int32_t> &EncodingSet = task.EncodingSet;
    std::map<int32_t, int32_t> &EncodingTree = task.EncodingTree;
    EncodingSet.clear();
    EncodingTree.clear();
    EncodingSet.resize(graph.freepolygonList.size(), -1);

    EncodingSet[task.OriginPolyIndex] = 0;
    EncodingTree[task.OriginPolyIndex] = -1;

    // int32_t Pnew = task.OriginPolyIndex;
    std::list<std::pair<int32_t, int32_t>> Qf; // PolyIndexSub, HomotopyPolyIndexPar
    for (int32_t PnearIndex : freepolygonList[task.OriginPolyIndex].polygonlink)
    {
        Qf.push_back({PnearIndex, task.OriginPolyIndex});
    }

    while (Qf.size())
    {
        int32_t PolyIndexSub = Qf.front().first;
        int32_t HomotopyPolyIndexPar = Qf.front().second;
        int32_t PolyIndexPar = HomotopyPolyIndexPar & 0x0000FFFF;
        Qf.pop_front();

        if (!THPPExpandingClassValidity(task, HomotopyPolyIndexPar, PolyIndexSub))
            continue;
        EncodingSet[PolyIndexSub]++;
        int32_t HomotopyPolyIndexSub = (EncodingSet[PolyIndexSub] << 16) | PolyIndexSub;
        EncodingTree[HomotopyPolyIndexSub] = HomotopyPolyIndexPar;
        // std::cout << "HomotopyPolyIndexSub: " << PolyIndexPar << std::endl;
        for (int32_t PnearIndex : freepolygonList[PolyIndexSub].polygonlink)
        {
            if (PnearIndex == PolyIndexPar)
                continue;
            Qf.push_back({PnearIndex, HomotopyPolyIndexSub});
        }
    }
    task.Init = task.Origin;
    task.HomotopyPolyIndex_Init = task.OriginPolyIndex;
}

bool BImap::THPPExpandingClassValidity(THPPtask &task, int32_t HomotopyPolyIndexPar, int32_t PolyIndexSub, double threshold)
{
    std::map<int32_t, int32_t> &EncodingTree = task.EncodingTree;
    BIgraph &graph = BIgraphList[task.graphIndex];
    std::vector<BIcutline> &cutlineList = graph.cutlineList;
    std::vector<BIfreepolygon> &freepolygonList = graph.freepolygonList;
    BIinvnodeMap &invnode2cutlineMap = graph.invnode2cutlineMap;

    int32_t PolyIndexPar = HomotopyPolyIndexPar & 0x0000FFFF;
    if (task.mod == 1 && PolyIndexSub == PolyIndexPar)
        return false;
    int32_t cutlineEIndex = invnode2cutlineMap[{PolyIndexPar, PolyIndexSub}];

    BIcutline &cutlineE = cutlineList[cutlineEIndex];
    std::vector<BIline> cpath;
    cpath.emplace_back(cutlineE.core, cutlineE.core);
    int32_t PathHomotopyPolyIndex_Now = HomotopyPolyIndexPar;
    int32_t PathHomotopyPolyIndex_Par = EncodingTree[PathHomotopyPolyIndex_Now];
    while (PathHomotopyPolyIndex_Par != -1)
    {
        int32_t PathPolyIndexPar = PathHomotopyPolyIndex_Par & 0x0000FFFF;
        int32_t PathPolyIndexNow = PathHomotopyPolyIndex_Now & 0x0000FFFF;
        if (task.mod == 1 && PolyIndexSub == PathPolyIndexPar)
            return false;
        int32_t cutlineIndex_Par2Now = invnode2cutlineMap[{PathPolyIndexPar, PathPolyIndexNow}];
        cpath.push_back(cutlineList[cutlineIndex_Par2Now].line);
        PathHomotopyPolyIndex_Now = PathHomotopyPolyIndex_Par;
        PathHomotopyPolyIndex_Par = EncodingTree[PathHomotopyPolyIndex_Now];
    }
    cpath.emplace_back(task.Origin, task.Origin);

    double CosttempM;
    std::list<BIpoint> Pathtemp;
    GetLeastHomotopyPath(cpath, Pathtemp, CosttempM);
    double TetherLength = task.TetherLength;

    if (CosttempM <= TetherLength)
        return true;
    // if (CosttempM + (cutlineE.line.S % cutlineE.line.E) / 2.0 > TetherLength)
    //     return false;
    if (CosttempM - (cutlineE.line.S % cutlineE.line.E) / 2.0 > TetherLength)
        return false;

    BIpoint S = cutlineE.line.S;
    BIpoint E = cutlineE.line.E;
    BIpoint S2E = E - S;
    double CosttempS, CosttempE;
    cpath.front().S = cpath.front().E = S;
    GetLeastHomotopyPath(cpath, Pathtemp, CosttempS);
    if (CosttempS <= TetherLength)
        return true;
    cpath.front().S = cpath.front().E = E;
    GetLeastHomotopyPath(cpath, Pathtemp, CosttempE);
    if (CosttempE <= TetherLength)
        return true;

    // 若单调则根据端点返回
    if (CosttempS <= CosttempM && CosttempM <= CosttempE && CosttempS > TetherLength)
        return false;
    if (CosttempS >= CosttempM && CosttempM >= CosttempE && CosttempE > TetherLength)
        return false;

    double low = 0.0;
    double high = 1.0;
    for (;;)
    {
        double mid1 = low + (high - low) / 3.0;
        double mid2 = high - (high - low) / 3.0;

        double CosttempM1;
        cpath.front().S = cpath.front().E = S + S2E * mid1;
        GetLeastHomotopyPath(cpath, Pathtemp, CosttempM1);
        if (CosttempM1 <= TetherLength)
            return true;
        double CosttempM2;
        cpath.front().S = cpath.front().E = S + S2E * mid2;
        GetLeastHomotopyPath(cpath, Pathtemp, CosttempM2);
        if (CosttempM2 <= TetherLength)
            return true;

        if (CosttempM1 < CosttempM2)
            high = mid2;
        else
            low = mid1;
        if (fabs(CosttempM1 - CosttempM2) < threshold)
            break;
    }
    return false;
}

void BImap::THPPgetCDTencoding(THPPtask &task, int32_t HomotopyPolyIndex, std::vector<int32_t> &polyPath)
{
    std::map<int32_t, int32_t> &EncodingTree = task.EncodingTree;
    BIgraph &graph = BIgraphList[task.graphIndex];
    std::vector<BIcutline> &cutlineList = graph.cutlineList;
    std::vector<BIfreepolygon> &freepolygonList = graph.freepolygonList;
    BIinvnodeMap &invnode2cutlineMap = graph.invnode2cutlineMap;

    int32_t PolyIndex = HomotopyPolyIndex & 0x0000FFFF;

    polyPath.clear();
    polyPath.push_back(PolyIndex);

    int32_t PathHomotopyPolyIndex_Now = HomotopyPolyIndex;
    int32_t PathHomotopyPolyIndex_Par = EncodingTree[PathHomotopyPolyIndex_Now];
    while (PathHomotopyPolyIndex_Par != -1)
    {
        int32_t PathPolyIndexPar = PathHomotopyPolyIndex_Par & 0x0000FFFF;

        polyPath.push_back(PathPolyIndexPar);

        PathHomotopyPolyIndex_Now = PathHomotopyPolyIndex_Par;
        PathHomotopyPolyIndex_Par = EncodingTree[PathHomotopyPolyIndex_Now];
    }
}

void BImap::THPPgetCDTencodingCutline(THPPtask &task, int32_t HomotopyPolyIndex, std::vector<BIline> &cpath, BIpoint goal)
{
    std::map<int32_t, int32_t> &EncodingTree = task.EncodingTree;
    BIgraph &graph = BIgraphList[task.graphIndex];
    std::vector<BIcutline> &cutlineList = graph.cutlineList;
    std::vector<BIfreepolygon> &freepolygonList = graph.freepolygonList;
    BIinvnodeMap &invnode2cutlineMap = graph.invnode2cutlineMap;

    int32_t PolyIndexPar = HomotopyPolyIndex & 0x0000FFFF;
    BIpoint Polycore = freepolygonList[PolyIndexPar].core;
    // int32_t cutlineEIndex = invnode2cutlineMap[{PolyIndexPar, PolyIndexSub}];

    cpath.clear();
    if (goal.x < 0 && goal.y < 0)
        cpath.emplace_back(Polycore, Polycore);
    else
        cpath.emplace_back(goal, goal);
    int32_t PathHomotopyPolyIndex_Now = HomotopyPolyIndex;
    int32_t PathHomotopyPolyIndex_Par = EncodingTree[PathHomotopyPolyIndex_Now];
    while (PathHomotopyPolyIndex_Par != -1)
    {
        int32_t PathPolyIndexPar = PathHomotopyPolyIndex_Par & 0x0000FFFF;
        int32_t PathPolyIndexNow = PathHomotopyPolyIndex_Now & 0x0000FFFF;
        int32_t cutlineIndex_Par2Now = invnode2cutlineMap[{PathPolyIndexPar, PathPolyIndexNow}];
        cpath.push_back(cutlineList[cutlineIndex_Par2Now].line);
        PathHomotopyPolyIndex_Now = PathHomotopyPolyIndex_Par;
        PathHomotopyPolyIndex_Par = EncodingTree[PathHomotopyPolyIndex_Now];
    }
    cpath.emplace_back(task.Origin, task.Origin);
}

double BImap::THPPoptimalPlanner(THPPtask &task, int32_t HomotopyPolyIndex_Init, int32_t &HomotopyPolyIndex_Goal,
                                 BIpoint Init, BIpoint Goal, std::list<BIpoint> &minPath)
{
    if (task.EncodingTree.count(HomotopyPolyIndex_Init) == 0)
    {
        printf("error: Tethered robots do not have this configuration (HomotopyPolyIndex)!!\r\n");
        return -1;
    }
    uint32_t anum_Goal = BIamap.at<uint16_t>(Goal.y, Goal.x);
    if (anum_Goal == 0xffff || anum_Goal != task.graphIndex)
    {
        printf("error: robot can't be here!!\r\n");
        return -1;
    }
    int32_t PolyIndex_Goal = BIimap.at<uint16_t>(Goal.y, Goal.x);
    if (task.EncodingSet[PolyIndex_Goal] == -1)
    {
        printf("error: Tether rope too short\r\n");
        return -1;
    }

    std::vector<int32_t> polyPathO;
    THPPgetCDTencoding(task, HomotopyPolyIndex_Init, polyPathO);
    BIgraph &graph = BIgraphList[task.graphIndex];
    std::vector<BIcutline> &cutlineList = graph.cutlineList;
    BIinvnodeMap &invnode2cutlineMap = graph.invnode2cutlineMap;
    double minCost = doubleMax;
    int32_t minHomotopyGoalIndex = -1;
    // std::list<BIpoint> minPath;

    for (int32_t HomotopyID = 0; HomotopyID <= task.EncodingSet[PolyIndex_Goal]; HomotopyID++)
    {
        std::vector<int32_t> polyPathG;
        int32_t HomotopyPolyIndex_goal = (HomotopyID << 16) | PolyIndex_Goal;
        THPPgetCDTencoding(task, HomotopyPolyIndex_goal, polyPathG);
        std::vector<BIline> cpath;
        cpath.emplace_back(task.Origin, task.Origin);
        int32_t polyPathOld = polyPathG.back();
        for (int32_t polyPathIndex = polyPathG.size() - 2; polyPathIndex >= 0; polyPathIndex--)
        {
            int32_t polyPathNew = polyPathG[polyPathIndex];
            int32_t cutlineIndex = invnode2cutlineMap[{polyPathOld, polyPathNew}];
            BIcutline &cutline = cutlineList[cutlineIndex];
            cpath.push_back(cutline.line);
            polyPathOld = polyPathNew;
        }
        cpath.emplace_back(Goal, Goal);
        double PathMinCost;
        std::list<BIpoint> PathTemp;
        GetLeastHomotopyPath(cpath, PathTemp, PathMinCost);
        if (PathMinCost > task.TetherLength)
            continue;
        /////////添加goal可行性的确认判断

        polyPathG.insert(polyPathG.end(), polyPathO.rbegin(), polyPathO.rend());
        ReversePathClearing(polyPathG);
        cpath.clear();
        cpath.emplace_back(Init, Init);
        polyPathOld = polyPathG.back();
        for (int32_t polyPathIndex = polyPathG.size() - 2; polyPathIndex >= 0; polyPathIndex--)
        {
            int32_t polyPathNew = polyPathG[polyPathIndex];
            int32_t cutlineIndex = invnode2cutlineMap[{polyPathOld, polyPathNew}];
            BIcutline &cutline = cutlineList[cutlineIndex];
            cpath.push_back(cutline.line);
            polyPathOld = polyPathNew;
        }
        cpath.emplace_back(Goal, Goal);
        GetLeastHomotopyPath(cpath, PathTemp, PathMinCost);
        if (PathMinCost < minCost)
        {
            minCost = PathMinCost;
            std::swap(minPath, PathTemp);
            HomotopyPolyIndex_Goal = HomotopyPolyIndex_goal;
        }
    }
    if (minCost < doubleMax)
        return minCost;
    printf("error: Tether rope too short\r\n");
    return -1;
}

double BImap::THPPoptimalPlanner(THPPtask &task)
{
    int32_t HomotopyPolyIndex_Goal;
    double cost = THPPoptimalPlanner(task, task.HomotopyPolyIndex_Init, HomotopyPolyIndex_Goal,
                                     task.Init, task.Goal, task.minPath);
    if (cost >= 0)
    {
        task.Init = task.Goal;
        task.HomotopyPolyIndex_Init = HomotopyPolyIndex_Goal;
        return cost;
    }

    return cost;
}

double BImap::UTHPPoptimalPlanner(THPPtask &task, BIpoint Init, BIpoint Goal, std::list<BIpoint> &minPath)
{
    uint32_t anum_Init = BIamap.at<uint16_t>(Init.y, Init.x);
    uint32_t anum_Goal = BIamap.at<uint16_t>(Goal.y, Goal.x);
    if (anum_Init == 0xffff || anum_Init != task.graphIndex ||
        anum_Goal == 0xffff || anum_Goal != task.graphIndex)
    {
        printf("error: robot can't be here!!\r\n");
        return -1;
    }

    int32_t PolyIndex_Init = BIimap.at<uint16_t>(Init.y, Init.x);
    int32_t PolyIndex_Goal = BIimap.at<uint16_t>(Goal.y, Goal.x);
    if (task.EncodingSet[PolyIndex_Init] == -1 || task.EncodingSet[PolyIndex_Goal] == -1)
    {
        printf("error: Tether rope too short\r\n");
        return -1;
    }

    BIgraph &graph = BIgraphList[task.graphIndex];
    std::vector<BIcutline> &cutlineList = graph.cutlineList;
    BIinvnodeMap &invnode2cutlineMap = graph.invnode2cutlineMap;

    double minCost = doubleMax;
    for (int32_t HomotopyIDS = 0; HomotopyIDS <= task.EncodingSet[PolyIndex_Init]; HomotopyIDS++)
    {
        std::vector<int32_t> polyPathS;
        int32_t HomotopyPolyIndex_Init = (HomotopyIDS << 16) | PolyIndex_Init;
        THPPgetCDTencoding(task, HomotopyPolyIndex_Init, polyPathS);
        for (int32_t HomotopyIDG = 0; HomotopyIDG <= task.EncodingSet[PolyIndex_Goal]; HomotopyIDG++)
        {
            std::vector<int32_t> polyPathG;
            int32_t HomotopyPolyIndex_Goal = (HomotopyIDG << 16) | PolyIndex_Goal;
            THPPgetCDTencoding(task, HomotopyPolyIndex_Goal, polyPathG);
            polyPathG.insert(polyPathG.end(), polyPathS.rbegin(), polyPathS.rend());
            ReversePathClearing(polyPathG);
            std::vector<BIline> cpath;
            cpath.emplace_back(Init, Init);
            int32_t polyPathOld = polyPathG.back();
            for (int32_t polyPathIndex = polyPathG.size() - 2; polyPathIndex >= 0; polyPathIndex--)
            {
                int32_t polyPathNew = polyPathG[polyPathIndex];
                int32_t cutlineIndex = invnode2cutlineMap[{polyPathOld, polyPathNew}];
                BIcutline &cutline = cutlineList[cutlineIndex];
                cpath.push_back(cutline.line);
                polyPathOld = polyPathNew;
            }
            cpath.emplace_back(Goal, Goal);

            double PathMinCost = doubleMax;
            std::list<BIpoint> PathTemp;
            GetLeastHomotopyPath(cpath, PathTemp, PathMinCost);
            if (PathMinCost < minCost)
            {
                minCost = PathMinCost;
                std::swap(minPath, PathTemp);
            }
        }
    }
    if (minCost < doubleMax)
        return minCost;
    printf("error: Init and Goal cannot connect OR Tether rope too short\r\n");
    return -1;
}

struct tmvNode
{
    int64_t par; // 0-15:HomotopyPolyIndex_Index(CDTencodingIndex)
                 // 16-31:goalIndex 通过0-31可获取并构建完整CDTencoding
                 // 32-47:标识（0-31对应CDTencoding在树中的重复数）
    double cost;
    double cost1;
};

double BImap::TMVoptimalPlanner(THPPtask &task, int32_t HomotopyPolyIndex_Init, BIpoint Init,
                                std::vector<BIpoint> Goals, std::list<BIpoint> &minPath)
{
    uint32_t compCount = 0;
    minPath.clear();
    if (task.EncodingTree.count(HomotopyPolyIndex_Init) == 0)
    {
        printf("error: Tethered robots do not have this configuration (HomotopyPolyIndex)!!\r\n");
        return -1;
    }
    std::vector<int32_t> polyPathS;
    THPPgetCDTencoding(task, HomotopyPolyIndex_Init, polyPathS);

    BIgraph &graph = BIgraphList[task.graphIndex];
    std::vector<BIcutline> &cutlineList = graph.cutlineList;
    BIinvnodeMap &invnode2cutlineMap = graph.invnode2cutlineMap;

    std::vector<std::vector<int32_t>> GoalsCDTEncodingIndexSet(Goals.size()); // 也许将被弃用
    std::vector<std::vector<std::vector<int32_t>>> GoalsCDTEncodingSet(Goals.size());
    for (int32_t goalIndex = 0; goalIndex < Goals.size(); goalIndex++)
    {
        BIpoint &goal = Goals[goalIndex];
        uint32_t anum_Goal = BIamap.at<uint16_t>(goal.y, goal.x);
        if (anum_Goal == 0xffff || anum_Goal != task.graphIndex)
        {
            printf("error: robot can't be here: (%lf, %lf)!!\r\n", goal.x, goal.y);
            return -1;
        }
        int32_t PolyIndex_Goal = BIimap.at<uint16_t>(goal.y, goal.x);
        if (task.EncodingSet[PolyIndex_Goal] == -1)
        {
            printf("error: Tether rope too short, can't be here: (%lf, %lf)!!\r\n", goal.x, goal.y);
            return -1;
        }
        GoalsCDTEncodingSet[goalIndex].reserve(task.EncodingSet[PolyIndex_Goal] + 1);
        for (int32_t HomotopyID = 0; HomotopyID <= task.EncodingSet[PolyIndex_Goal]; HomotopyID++)
        {
            std::vector<int32_t> polyPathG;
            int32_t HomotopyPolyIndex_goal = (HomotopyID << 16) | PolyIndex_Goal;
            THPPgetCDTencoding(task, HomotopyPolyIndex_goal, polyPathG);
            std::vector<BIline> cpath;
            THPPgetCDTencodingCutline(task, HomotopyPolyIndex_goal, cpath, goal);
            // cpath.emplace_back(task.Origin, task.Origin);
            // int32_t polyPathOld = polyPathG.back();
            // for (int32_t polyPathIndex = polyPathG.size() - 2; polyPathIndex >= 0; polyPathIndex--)
            // {
            //     int32_t polyPathNew = polyPathG[polyPathIndex];
            //     int32_t cutlineIndex = invnode2cutlineMap[{polyPathOld, polyPathNew}];
            //     BIcutline &cutline = cutlineList[cutlineIndex];
            //     cpath.push_back(cutline.line);
            //     polyPathOld = polyPathNew;
            // }
            // cpath.emplace_back(goal, goal);
            double PathMinCost;
            std::list<BIpoint> PathTemp;
            GetLeastHomotopyPath(cpath, PathTemp, PathMinCost);
            if (PathMinCost > task.TetherLength)
                continue;
            GoalsCDTEncodingIndexSet[goalIndex].push_back(HomotopyPolyIndex_goal);
            GoalsCDTEncodingSet[goalIndex].push_back(polyPathG);

            compCount++;
        }
        if (GoalsCDTEncodingSet[goalIndex].size() == 0)
        {
            printf("error: Tether rope too short, can't be here: (%lf, %lf)!!\r\n", goal.x, goal.y);
            return -1;
        }
        // std::cout << "GoalsCDTEncodingSet[goalIndex].size(): " << GoalsCDTEncodingSet[goalIndex].size() << std::endl;
    }

    std::map<int64_t, tmvNode> tmvTree;
    std::map<int32_t, int32_t> numCDTencoding; // The max number of 0-31 CDTencoding of tmvNode
                                               // 即32-47:标识（0-31对应CDTencoding在树中的重复数）
    std::priority_queue<std::pair<double, int64_t>,
                        std::vector<std::pair<double, int64_t>>,
                        std::greater<std::pair<double, int64_t>>>
        tmvQ; //{cost, tmvNodeID}

    for (int32_t EncodingIndex = 0; EncodingIndex < GoalsCDTEncodingSet[0].size(); EncodingIndex++)
    {
        // int32_t CDTencodingID = EncodingIndex | (0 << 16);
        // int32_t CDTencodingcount;
        // if (numCDTencoding.count(CDTencodingID))
        //     CDTencodingcount = ++numCDTencoding[CDTencodingID];
        // else
        //     numCDTencoding[CDTencodingID] = CDTencodingcount = 0;
        // int64_t tmvNodeID = CDTencodingID | (CDTencodingcount << 32);
        // 优化后
        numCDTencoding[EncodingIndex] = 0;
        int64_t tmvNodeID = EncodingIndex;

        std::vector<int32_t> &polyPathG = GoalsCDTEncodingSet[0][EncodingIndex];
        std::list<BIpoint> PathTemp;
        double PathMinCost = THPPoptimalReConfig(task, polyPathS, polyPathG,
                                                 Init, Goals[0], PathTemp);
        compCount++;

        tmvNode node;
        node.par = -1;
        node.cost1 = PathMinCost;
        node.cost = 2 * PathMinCost;
        tmvTree[tmvNodeID] = node;
        tmvQ.push({node.cost, tmvNodeID});
    }
    // std::cout << "tmvQ.size(): " << tmvQ.size() << std::endl;
    while (tmvQ.size())
    {
        int64_t tmvNodeID = tmvQ.top().second;
        tmvNode node = tmvTree[tmvNodeID];
        tmvQ.pop();

        int32_t nodeGoalIndex = (tmvNodeID >> 16) & 0xFFFF;
        int32_t nodeGoalIndexNext = nodeGoalIndex + 1;

        if (nodeGoalIndexNext == Goals.size())
        {
            std::list<BIpoint> PathTemp;
            int32_t EncodingIndex = tmvNodeID & 0xFFFF;
            double minCost;
            minCost = THPPoptimalReConfig(task, GoalsCDTEncodingSet[nodeGoalIndex][EncodingIndex],
                                          polyPathS, Goals[nodeGoalIndex], Init, PathTemp);
            minPath.splice(minPath.begin(), PathTemp);
            while (node.par != -1)
            {
                int64_t tmvNodeIDPar = node.par;
                int32_t EncodingIndexPar = tmvNodeIDPar & 0xFFFF;
                int32_t nodeGoalIndexPar = (tmvNodeIDPar >> 16) & 0xFFFF;

                minCost += THPPoptimalReConfig(task, GoalsCDTEncodingSet[nodeGoalIndexPar][EncodingIndexPar],
                                    GoalsCDTEncodingSet[nodeGoalIndex][EncodingIndex],
                                    Goals[nodeGoalIndexPar], Goals[nodeGoalIndex], PathTemp);
                minPath.splice(minPath.begin(), PathTemp);

                tmvNodeID = tmvNodeIDPar;
                EncodingIndex = EncodingIndexPar;
                nodeGoalIndex = nodeGoalIndexPar;
                node = tmvTree[tmvNodeID];

                compCount++;
            }
            minCost += THPPoptimalReConfig(task, polyPathS, GoalsCDTEncodingSet[nodeGoalIndex][EncodingIndex],
                                Init, Goals[0], PathTemp);

            compCount += 2;
            minPath.splice(minPath.begin(), PathTemp);
            // std::cout << "compCount: " << compCount << std::endl;
            return minCost;
        }

        int32_t EncodingIndex_k = tmvNodeID & 0xFFFF;
        std::vector<int32_t> &polyPathG_k = GoalsCDTEncodingSet[nodeGoalIndex][EncodingIndex_k];

        for (int32_t EncodingIndex_k1 = 0;
             EncodingIndex_k1 < GoalsCDTEncodingSet[nodeGoalIndexNext].size();
             EncodingIndex_k1++)
        {
            int32_t CDTencodingID = EncodingIndex_k1 | (nodeGoalIndexNext << 16);
            int32_t CDTencodingcount;
            if (numCDTencoding.count(CDTencodingID))
                CDTencodingcount = ++numCDTencoding[CDTencodingID];
            else
                numCDTencoding[CDTencodingID] = CDTencodingcount = 0;
            int64_t tmvNodeIDNext = CDTencodingID | ((int64_t)CDTencodingcount << 32);
            std::vector<int32_t> &polyPathG_k1 = GoalsCDTEncodingSet[nodeGoalIndexNext][EncodingIndex_k1];

            double Cost_kk1 = doubleMax;
            double Cost_k1s = doubleMax;
            {
                std::list<BIpoint> PathTemp;
                Cost_kk1 = THPPoptimalReConfig(task, polyPathG_k, polyPathG_k1,
                                               Goals[nodeGoalIndex], Goals[nodeGoalIndexNext], PathTemp);
                Cost_k1s = THPPoptimalReConfig(task, polyPathS, polyPathG_k1,
                                               Init, Goals[nodeGoalIndexNext], PathTemp);
                compCount += 2;
            }
            tmvNode nodeNext;
            nodeNext.par = tmvNodeID;
            nodeNext.cost1 = node.cost1 + Cost_kk1;
            nodeNext.cost = nodeNext.cost1 + Cost_k1s;
            tmvTree[tmvNodeIDNext] = nodeNext;
            tmvQ.push({nodeNext.cost, tmvNodeIDNext});
        }
    }
    printf("error: no found!!!\r\n");
    return -1;
}

double BImap::TMVoptimalPlannerViolent(THPPtask &task, int32_t HomotopyPolyIndex_Init, BIpoint Init,
                                     std::vector<BIpoint> Goals, std::list<BIpoint> &minPath)
{
    uint32_t compCount = 1;
    minPath.clear();
    if (task.EncodingTree.count(HomotopyPolyIndex_Init) == 0)
    {
        printf("error: Tethered robots do not have this configuration (HomotopyPolyIndex)!!\r\n");
        return -1;
    }
    std::vector<int32_t> polyPathS;
    THPPgetCDTencoding(task, HomotopyPolyIndex_Init, polyPathS);

    BIgraph &graph = BIgraphList[task.graphIndex];
    std::vector<BIcutline> &cutlineList = graph.cutlineList;
    BIinvnodeMap &invnode2cutlineMap = graph.invnode2cutlineMap;

    std::vector<std::vector<std::vector<int32_t>>> GoalsCDTEncodingSet(Goals.size());
    for (int32_t goalIndex = 0; goalIndex < Goals.size(); goalIndex++)
    {
        BIpoint &goal = Goals[goalIndex];
        uint32_t anum_Goal = BIamap.at<uint16_t>(goal.y, goal.x);
        if (anum_Goal == 0xffff || anum_Goal != task.graphIndex)
        {
            printf("error: robot can't be here: (%lf, %lf)!!\r\n", goal.x, goal.y);
            return -1;
        }
        int32_t PolyIndex_Goal = BIimap.at<uint16_t>(goal.y, goal.x);
        if (task.EncodingSet[PolyIndex_Goal] == -1)
        {
            printf("error: Tether rope too short, can't be here: (%lf, %lf)!!\r\n", goal.x, goal.y);
            return -1;
        }
        GoalsCDTEncodingSet[goalIndex].reserve(task.EncodingSet[PolyIndex_Goal] + 1);
        for (int32_t HomotopyID = 0; HomotopyID <= task.EncodingSet[PolyIndex_Goal]; HomotopyID++)
        {
            std::vector<int32_t> polyPathG;
            int32_t HomotopyPolyIndex_goal = (HomotopyID << 16) | PolyIndex_Goal;
            THPPgetCDTencoding(task, HomotopyPolyIndex_goal, polyPathG);
            std::vector<BIline> cpath;
            THPPgetCDTencodingCutline(task, HomotopyPolyIndex_goal, cpath, goal);

            double PathMinCost;
            std::list<BIpoint> PathTemp;
            GetLeastHomotopyPath(cpath, PathTemp, PathMinCost);
            if (PathMinCost > task.TetherLength)
                continue;
            GoalsCDTEncodingSet[goalIndex].push_back(polyPathG);

            // compCount++;
        }
        if (GoalsCDTEncodingSet[goalIndex].size() == 0)
        {
            printf("error: Tether rope too short, can't be here: (%lf, %lf)!!\r\n", goal.x, goal.y);
            return -1;
        }
        compCount *= GoalsCDTEncodingSet[goalIndex].size();
        // std::cout << "GoalsCDTEncodingSet[goalIndex].size(): " << GoalsCDTEncodingSet[goalIndex].size() << std::endl;
    }
    // std::cout << "compCount: " << compCount << std::endl;

    compCount = 0;
    std::vector<int32_t> ArrangementCounter(Goals.size(), 0);
    double costMin = doubleMax;
    bool runkey = true;
    while (runkey)
    {
        compCount++;
        double costNew = 0;
        std::list<BIpoint> PathNew;
        std::list<BIpoint> PathTemp;
        costNew += THPPoptimalReConfig(task, polyPathS, GoalsCDTEncodingSet[0][ArrangementCounter[0]],
                                       Init, Goals[0], PathTemp);
        PathNew.splice(PathNew.end(), PathTemp);
        for (int32_t goalIndex = 1; goalIndex < Goals.size(); goalIndex++)
        {
            costNew += THPPoptimalReConfig(task,
                                           GoalsCDTEncodingSet[goalIndex - 1][ArrangementCounter[goalIndex - 1]],
                                           GoalsCDTEncodingSet[goalIndex][ArrangementCounter[goalIndex]],
                                           Goals[goalIndex - 1], Goals[goalIndex], PathTemp);
            PathNew.splice(PathNew.end(), PathTemp);
        }
        costNew += THPPoptimalReConfig(task, GoalsCDTEncodingSet[Goals.size() - 1][ArrangementCounter[Goals.size() - 1]],
                                       polyPathS, Goals[Goals.size() - 1], Init, PathTemp);
        PathNew.splice(PathNew.end(), PathTemp);
        if (costNew < costMin)
        {
            costMin = costNew;
            std::swap(minPath, PathNew);
        }

        int32_t Carryer = 0;
        while (true)
        {
            if (++ArrangementCounter[Carryer] >= GoalsCDTEncodingSet[Carryer].size())
            {
                ArrangementCounter[Carryer] = 0;
                Carryer++;
                if (Carryer >= Goals.size())
                {
                    runkey = false;
                    break;
                }
            }
            else
                break;
        }
    }
    // std::cout << "compCount: " << compCount << std::endl;

    return costMin;
}

double BImap::THPPoptimalReConfig(THPPtask &task, std::vector<int32_t> &polyPathS, std::vector<int32_t> &polyPathG,
                                  BIpoint Init, BIpoint Goal, std::list<BIpoint> &minPath)
{
    BIgraph &graph = BIgraphList[task.graphIndex];
    std::vector<BIcutline> &cutlineList = graph.cutlineList;
    BIinvnodeMap &invnode2cutlineMap = graph.invnode2cutlineMap;

    minPath.clear();
    std::vector<int32_t> polyPathTemp = polyPathG;
    polyPathTemp.insert(polyPathTemp.end(), polyPathS.rbegin(), polyPathS.rend());
    ReversePathClearing(polyPathTemp);
    std::vector<BIline> cpath;
    cpath.emplace_back(Init, Init);
    int32_t polyPathOld = polyPathTemp.back();
    for (int32_t polyPathIndex = polyPathTemp.size() - 2; polyPathIndex >= 0; polyPathIndex--)
    {
        int32_t polyPathNew = polyPathTemp[polyPathIndex];
        int32_t cutlineIndex = invnode2cutlineMap[{polyPathOld, polyPathNew}];
        BIcutline &cutline = cutlineList[cutlineIndex];
        cpath.push_back(cutline.line);
        polyPathOld = polyPathNew;
    }
    cpath.emplace_back(Goal, Goal);

    double minPathCost = doubleMax;
    GetLeastHomotopyPath(cpath, minPath, minPathCost);

    return minPathCost;
}

void BImap::GetAllOptConfigurations(THPPtask &task, BIpoint goal,
                                    std::map<int32_t, std::pair<std::list<BIpoint>, double>>
                                        &ConfigList)
{
    ConfigList.clear();
    int32_t PolyIndex = BIimap.at<uint16_t>(goal.y, goal.x);
    for (int32_t i = 0; i <= task.EncodingSet[PolyIndex]; i++)
    {
        std::vector<BIline> cpath;
        int32_t HPolyIndex = (i << 16) | PolyIndex;
        THPPgetCDTencodingCutline(task, HPolyIndex, cpath, goal);
        std::list<BIpoint> Pathtemp;
        double Costtemp;
        GetLeastHomotopyPath(cpath, Pathtemp, Costtemp);
        if (Costtemp > task.TetherLength)
            continue;
        ConfigList[HPolyIndex].second = Costtemp;
        std::swap(ConfigList[HPolyIndex].first, Pathtemp);
    }
}