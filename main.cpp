#include <iostream>
#include <vector>
#include <string>
#include "wpml_parser.h"  // 包含我们自定义的头文件

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " path/to/waylines.wpml\n";
        return 1;
    }

    // 调用解析函数
    std::string wpmlFile = argv[1];
    std::vector<PlanLineData> lines = parseWaylinesWPML(wpmlFile);
    std::cout << "Found " << lines.size() << " Folder(s) in " << wpmlFile << "\n";

    // 简单打印结果
    for (size_t i = 0; i < lines.size(); ++i) {
        const auto& L = lines[i];
        std::cout << "=== PlanLineData #" << i << " ===\n";
        std::cout << "  finishedAction = " << L.finishedaction() << "\n";
        std::cout << "  loseAction     = " << L.loseaction() << "\n";
        std::cout << "  homeHeight     = " << L.homeheight() << "\n";
        std::cout << "  maxSpeed       = " << L.maxspeed() << "\n";
        std::cout << "  autoSpeed      = " << L.autospeed() << "\n";
        std::cout << "  securityHeight = " << L.securityheight() << "\n";
        std::cout << "  isSaveEnergy   = " << L.issaveenergymode() << "\n";
        std::cout << "  templateId     = " << L.templateid() << "\n";
        std::cout << "  Points count   = " << L.points_size() << "\n";

        for (int p = 0; p < L.points_size(); ++p) {
            const auto& pt = L.points(p);
            std::cout << "    [Waypoint " << p << "] "
                      << "lng=" << pt.lng()
                      << ", lat=" << pt.lat()
                      << ", height=" << pt.height()
                      << ", speed=" << pt.speed()
                      << ", flightPathMode=" << pt.flightpathmode()
                      << ", headingMode=" << pt.headingmode()
                      << ", actions=" << pt.actions_size()
                      << "\n";

            // 如需打印每个动作详情
            for (int a = 0; a < pt.actions_size(); ++a) {
                const auto& act = pt.actions(a);
                std::cout << "       -> Action #" << a 
                          << " type=" << act.type() 
                          << ", param=" << act.param()
                          << ", wait=" << act.waittime()
                          << ", speakInfo=" << act.speakinfo()
                          << "\n";
            }
        }
        std::cout << "\n";
    }

    return 0;
}
