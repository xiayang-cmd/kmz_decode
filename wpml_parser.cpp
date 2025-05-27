#include "wpml_parser.h"
#include <iostream>       // 可能用到输出日志
#include <string>
#include <vector>
#include <cstdio>         // sscanf
#include <pugixml.hpp>    // 解析 XML

// -------------- 工具函数：WPML -> Protobuf 映射 -------------- //

// 航线结束动作: WPML finishAction -> Protobuf finishedAction
// goHome/noAction/autoLand/gotoFirstWaypoint -> 1/2/3/5 (仅示例)
static uint32_t mapFinishAction(const std::string& actionStr)
{
    if (actionStr == "goHome") {
        return 1; // 1=返航
    } else if (actionStr == "noAction") {
        return 2; // 2=原地悬停(示例用)
    } else if (actionStr == "autoLand") {
        return 3; // 3=原地降落
    } else if (actionStr == "gotoFirstWaypoint") {
        return 5; // 5=返回第一个航点
    }
    // 默认
    return 2;
}

// 失控动作: WPML exitOnRCLost -> loseAction(0=返航,1=继续)
static uint32_t mapLoseAction(const std::string& exitOnRCLost)
{
    if (exitOnRCLost == "goContinue") {
        return 1; 
    }
    // "executeLostAction" 或其他 => 0=返航
    return 0;
}

// flyToWaylineMode -> isSaveEnergyMode(仅示例: pointToPoint=1,其它=0)
static uint32_t mapSaveEnergyMode(const std::string& flyModeStr)
{
    if (flyModeStr == "pointToPoint") {
        return 1; 
    }
    return 0;
}

// 航点拐弯模式 -> flightPathMode
static uint32_t mapFlightPathMode(const std::string& turnModeStr)
{
    // WPML 常见: toPointAndStopWithDiscontinuityCurvature, toPointAndPassWithDiscontinuityCurvature
    // 这里示例：3=曲线(停),4=曲线(不停). 如果没有指定, 就当直线=1
    if (turnModeStr == "toPointAndStopWithContinuityCurvature") {
        return 3;
    } else if (turnModeStr == "toPointAndPassWithContinuityCurvature") {
        return 4;
    } else if (turnModeStr == "coordinateTurn") {
        return 2; // 2=协调转弯
    }
    return 1; // 默认为直线
}

// 航点航向模式 -> headingMode
static uint32_t mapHeadingMode(const std::string& headingModeStr)
{
    // followWayline：飞机机头跟随航线方向飞往下一个航点 => 0=自动
    if (headingModeStr == "followWayline") {
        return 0; 
    }
    // manually：飞行过程中用户可手动控制机头 => 2=遥控器控制
    else if (headingModeStr == "manually") {
        return 2;
    }
    // fixed：执行航点操作后，飞机机头保持此前飞往下一个航点的偏航角 => 1=锁定
    else if (headingModeStr == "fixed") {
        return 1;
    }
    // smoothTransition：航点目标偏航角由 waypointHeadingAngle 指定，均匀过渡 => 3=机头依照航点偏航设置方向旋转
    else if (headingModeStr == "smoothTransition") {
        return 3;
    }
    // towardPOI：飞机航向面向兴趣点 => 4=朝向兴趣点
    else if (headingModeStr == "towardPOI") {
        return 4;
    }

    // 其他情况缺省为 0=自动，或根据需要自定义
    return 0;
}

// 根据 <wpml:actionActuatorFunc> 等解析动作
static PointAction parseAction(const pugi::xml_node& actionNode)
{
    PointAction pAction;
    pAction.set_type(0);
    pAction.set_param(0.f);
    pAction.set_waittime(0);
    pAction.clear_speakinfo();

    // 功能名称 与 参数节点
    const std::string func = actionNode.child_value("wpml:actionActuatorFunc");
    const auto paramNode  = actionNode.child("wpml:actionActuatorFuncParam");

    /* ────────── 1  变焦  zoom ────────── */
    if (func == "zoom")
    {   // focalLength 与变焦倍率是 1 ~ 1 映射；有时字段叫 <wpml:focalLength> 或 <wpml:zoomRatio>（这个好像没有）
        float zoomVal = 0.f;
        if (paramNode)
        {
            if (auto n = paramNode.child("wpml:focalLength"))
                zoomVal = n.text().as_float();
            else if (auto n = paramNode.child("wpml:zoomRatio"))
                zoomVal = n.text().as_float();
        }
        pAction.set_type(1);          // 1 = 变焦
        pAction.set_param(zoomVal);   // 2~200；由上层业务校验合法区间
    }
    /* ────────── 2  拍照  takePhoto ────────── */
    else if (func == "takePhoto")
    {   // 相机索引或拍照类型
        int payloadIdx = 1;
        if (paramNode)
        {
            if (auto n = paramNode.child("wpml:payloadIndex"))
                payloadIdx = n.text().as_int();
            else if (auto n = paramNode.child("wpml:photoType"))
                payloadIdx = n.text().as_int();     // DJI 31.0 固件里叫 photoType
        }
        pAction.set_type(2);
        pAction.set_param(static_cast<float>(payloadIdx));    // 1~7
    }
    /* ────────── 3/4  录像开始 / 停止 ────────── */
    else if (func == "startRecord")
    {
        pAction.set_type(3);   // 3 = 录像
    }
    else if (func == "stopRecord")
    {
        pAction.set_type(4);   // 4 = 停录
    }
    /* ────────── 5  飞机机头偏航 rotateYaw ────────── */
    else if (func == "rotateYaw")
    {   // 角度 -180~180
        float yaw = paramNode.child("wpml:aircraftHeading").text().as_float();
        pAction.set_type(5);
        pAction.set_param(yaw);
    }
    /* ────────── 6/7/8  云台三轴 gimbalRotate ────────── */
    else if (func == "gimbalRotate")
    {
        bool pitchE = paramNode.child("wpml:gimbalPitchRotateEnable").text().as_int() == 1;
        bool yawE   = paramNode.child("wpml:gimbalYawRotateEnable").text().as_int()   == 1;
        bool rollE  = paramNode.child("wpml:gimbalRollRotateEnable").text().as_int()  == 1;

        if (pitchE)
        {
            pAction.set_type(6);
            pAction.set_param(paramNode.child("wpml:gimbalPitchRotateAngle").text().as_float());
        }
        else if (yawE)
        {
            pAction.set_type(7);
            pAction.set_param(paramNode.child("wpml:gimbalYawRotateAngle").text().as_float());
        }
        else if (rollE)
        {
            pAction.set_type(8);
            pAction.set_param(paramNode.child("wpml:gimbalRollRotateAngle").text().as_float());
        }
    }
    /* ────────── 9  悬停 hover ────────── */
    else if (func == "hover")
    {
        float sec = paramNode.child("wpml:hoverTime").text().as_float();
        pAction.set_type(9);
        pAction.set_param(sec);      // 1–25 s
    }
    /* ────────── 10/11/12  等距 / 等时 / 结束间隔拍照 ────────── */
    else if (func == "startIntervalPhotoByDistance")
    {
        pAction.set_type(10);
        pAction.set_param(paramNode.child("wpml:distance").text().as_float());
    }
    else if (func == "startIntervalPhotoByTime")
    {
        pAction.set_type(11);
        pAction.set_param(paramNode.child("wpml:timeInterval").text().as_float());
    }
    else if (func == "stopIntervalPhoto")
    {
        pAction.set_type(12);
    }
    /* ────────── 13/14/15  喊话（单次 / 开始循环 / 结束循环）────────── */
    else if (func == "singleSpeak")
    {
        pAction.set_type(13);
        if (paramNode)
        {
            std::string vol  = paramNode.child("wpml:volume").text().get();
            std::string mode = paramNode.child("wpml:speakMode").text().get(); // 0:tts 1:localFile ...
            std::string text = paramNode.child("wpml:speakContent").text().get();
            pAction.set_speakinfo(vol + "_" + mode + "_" + text);
        }
    }
    else if (func == "startLoopSpeak")
    {
        pAction.set_type(14);
        // speakInfo 同 singleSpeak
    }
    else if (func == "stopLoopSpeak")
    {
        pAction.set_type(15);
    }
    /* ────────── 16  对焦 focus ────────── */
    else if (func == "focus")
    {
        float focusDist = paramNode.child("wpml:focusDistance").text().as_float(0.f);
        pAction.set_type(16);
        pAction.set_param(focusDist);
    }
    /* ────────── 17  切换视频源 switchVideoSource ────────── */
    else if (func == "switchVideoSource")
    {
        int src = paramNode.child("wpml:videoSource").text().as_int(1); // 1 广角 / 2 变焦 / 3 红外
        pAction.set_type(17);
        pAction.set_param(static_cast<float>(src));
    }
    /* ────────── 其它未列举的 Func，可按需扩展 ────────── */

    return pAction;
}


// -------------- 主解析函数：返回多个航线 -------------- //
// 如果只有一条航线，可直接返回 PlanLineData
std::vector<PlanLineData> parseWaylinesWPML(const std::string& xmlPath)
{
    std::vector<PlanLineData> planLines;

    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(xmlPath.c_str());
    if (!result) {
        std::cerr << "Failed to load or parse: " << xmlPath << "\n";
        return planLines;
    }

    // 先读取 <wpml:missionConfig> (可作为全局默认)
    // 如需多条航线使用相同默认值，可先存在一个临时对象里
    uint32_t defaultFinishedAction = 2; // noAction->2
    uint32_t defaultLoseAction = 0;     // executeLostAction->0
    uint32_t defaultHomeHeight = 50;
    float    defaultMaxSpeed   = 15.0f;
    float    defaultAutoSpeed  = 8.0f;
    uint32_t defaultSecurityHeight = 20;
    uint32_t defaultSaveEnergy = 0;

    pugi::xml_node missionConfig = doc.select_node("//*[local-name()='missionConfig']").node();
    if (missionConfig) {
        // finishAction
        std::string finishActStr = missionConfig.child_value("wpml:finishAction");
        if (!finishActStr.empty()) {
            defaultFinishedAction = mapFinishAction(finishActStr);
        }
        // globalRTHHeight
        if (missionConfig.child("wpml:globalRTHHeight")) {
            defaultHomeHeight = missionConfig.child("wpml:globalRTHHeight").text().as_uint();
        }
        // takeOffSecurityHeight
        if (missionConfig.child("wpml:takeOffSecurityHeight")) {
            defaultSecurityHeight = missionConfig.child("wpml:takeOffSecurityHeight").text().as_uint();
        }
        // exitOnRCLost
        std::string rcLostStr = missionConfig.child_value("wpml:exitOnRCLost");
        if (!rcLostStr.empty()) {
            defaultLoseAction = mapLoseAction(rcLostStr);
        }
        // flyToWaylineMode
        std::string flyModeStr = missionConfig.child_value("wpml:flyToWaylineMode");
        if (!flyModeStr.empty()) {
            defaultSaveEnergy = mapSaveEnergyMode(flyModeStr);
        }
        // globalTransitionalSpeed => 作为 maxSpeed
        if (missionConfig.child("wpml:globalTransitionalSpeed")) {
            defaultMaxSpeed = missionConfig.child("wpml:globalTransitionalSpeed").text().as_float();
        }
    }

    // 遍历所有 <Folder>，每个 Folder 看成一条航线
    auto folderNodes = doc.select_nodes("//*[local-name()='Folder']");
    for (auto& fn : folderNodes) {
        pugi::xml_node folder = fn.node();

        // 新建一个 PlanLineData,填入默认值
        PlanLineData line;
        line.set_finishedaction(defaultFinishedAction);
        line.set_loseaction(defaultLoseAction);
        line.set_homeheight(defaultHomeHeight);
        line.set_maxspeed(defaultMaxSpeed);
        line.set_autospeed(defaultAutoSpeed);
        line.set_securityheight(defaultSecurityHeight);
        line.set_issaveenergymode(defaultSaveEnergy);

        // 从 Folder 级别覆盖
        // templateId
        if (folder.child("wpml:templateId")) {
            line.set_templateid(folder.child("wpml:templateId").text().as_uint());
        }
        // autoFlightSpeed -> autospeed
        if (folder.child("wpml:autoFlightSpeed")) {
            line.set_autospeed(folder.child("wpml:autoFlightSpeed").text().as_float());
        }

        // 遍历 <Placemark>，解析航点
        auto placemarkNodes = folder.select_nodes(".//*[local-name()='Placemark']");
        for (auto& pmn : placemarkNodes) {
            pugi::xml_node pm = pmn.node();

            PointData pt;
            pt.set_speed(line.autospeed()); // 先默认等于 line.autospeed()
            pt.set_flightpathmode(1);       // 缺省直线
            pt.set_headingmode(0);         // 缺省自动

            // 解析经纬度: <coordinates>lon,lat</coordinates>
            std::string coordStr = pm.child("Point").child_value("coordinates");
            if (!coordStr.empty()) {
                double lon=0.0, lat=0.0;
                // 注意有些情况有3个值(lon,lat,alt),本例只解析lon,lat
                sscanf(coordStr.c_str(), "%lf,%lf", &lon, &lat);
                pt.set_lng(lon);
                pt.set_lat(lat);
            }

            // <wpml:executeHeight>
            if (pm.child("wpml:executeHeight")) {
                pt.set_height(pm.child("wpml:executeHeight").text().as_float());
            }

            // <wpml:waypointSpeed>
            if (pm.child("wpml:waypointSpeed")) {
                pt.set_speed(pm.child("wpml:waypointSpeed").text().as_float());
            }

            // 拐弯参数: <wpml:waypointTurnParam>
            pugi::xml_node turnParam = pm.child("wpml:waypointTurnParam");
            if (turnParam) {
                std::string turnModeStr = turnParam.child_value("wpml:waypointTurnMode");
                pt.set_flightpathmode(mapFlightPathMode(turnModeStr));
                if (turnParam.child("wpml:waypointTurnDampingDist")) {
                    pt.set_dampingdistance(turnParam.child("wpml:waypointTurnDampingDist").text().as_float());
                }
            }

            // 航向参数: <wpml:waypointHeadingParam>
            pugi::xml_node headingParam = pm.child("wpml:waypointHeadingParam");
            if (headingParam) {
                std::string headingModeStr = headingParam.child_value("wpml:waypointHeadingMode");
                pt.set_headingmode(mapHeadingMode(headingModeStr));
                // 如果有具体偏航角度,可在 headingParam 中加自定义字段
                if (headingParam.child("wpml:waypointHeadingAngle")) {
                    pt.set_heading(headingParam.child("wpml:waypointHeadingAngle").text().as_float());
                }
            }

            // 动作: <wpml:actionGroup>/<wpml:action>
            // 可能有多个 <actionGroup>,每个 <actionGroup> 里有多个 <action>
            auto actionGroupNodes = pm.select_nodes(".//*[local-name()='actionGroup']");
            for (auto& agn : actionGroupNodes) {
                pugi::xml_node actionGroup = agn.node();
                // 遍历其下的 <action>
                for (auto& an : actionGroup.select_nodes(".//*[local-name()='action']")) {
                    pugi::xml_node actionNode = an.node();
                    PointAction pAct = parseAction(actionNode);
                    // 也可根据 <wpml:actionTrigger> 处理触发逻辑
                    // 例如 reachPoint / delayTime / ...
                    // 这里暂不深入

                    // 加入 pt 的 actions
                    pt.add_actions()->CopyFrom(pAct);
                }
            }

            // 最后将航点加入 line
            line.add_points()->CopyFrom(pt);
        }

        // 将该条航线加入 planLines
        planLines.push_back(line);
    }

    return planLines;
}