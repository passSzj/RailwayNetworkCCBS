#include <iostream>
#include "map.h"
#include "heuristic.h"
#include "task.h"
#include "windows.h"
#include <psapi.h>
#include "sipp.h"

#include "CBS.h"

void print_memory_usage();


int main() {
    Map map;
    Task task;
    map.getMap("../MapData/Task3.xml");
    task.getTask("../AgentData/Task3_10Agent.xml");
    task.getXY(map);

    CBS cbs;
    Solution solution = cbs.findSolution(map,task);
    auto found = solution.found?"true":"false";
    std::cout<< "Soulution found: " << found << "\nRuntime: "<<solution.time.count() << "\nMakespan: " << solution.makespan << "\nFlowtime: " << solution.flowtime<< "\nInitial Cost: "<<solution.initCost
             << "\nHL expanded: " << solution.high_level_expanded << "\nLL expanded(avg): " << solution.low_level_expanded << std::endl;

    print_memory_usage();
}




void print_memory_usage() {
    PROCESS_MEMORY_COUNTERS_EX pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc))) {
        SIZE_T physMemUsedByMe = pmc.WorkingSetSize;
        SIZE_T physMemPeakUsedByMe = pmc.PeakWorkingSetSize;
        SIZE_T virtualMemUsedByMe = pmc.PrivateUsage;

        std::cout << "Memory Usage:" << std::endl;
        std::cout << "Physical Memory Used: " << physMemUsedByMe / 1024 << " KB" << std::endl;
        std::cout << "Peak Physical Memory Used: " << physMemPeakUsedByMe / 1024 << " KB" << std::endl;
        std::cout << "Virtual Memory Used: " << virtualMemUsedByMe / 1024 << " KB" << std::endl;
    } else {
        std::cerr << "Failed to retrieve memory usage information" << std::endl;
    }
}
