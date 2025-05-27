#ifndef WPML_PARSER_H
#define WPML_PARSER_H

#include <string>
#include <vector>

// 注意：这里包含生成的 Protobuf 头文件，
// 使得 parseWaylinesWPML 可以返回 std::vector<PlanLineData>
#include "TelemetryDataBuf-new.pb.h"

// 对外提供的解析函数接口
// 传入 WPML 文件路径，返回多条航线数据
std::vector<PlanLineData> parseWaylinesWPML(const std::string& xmlPath);

#endif // WPML_PARSER_H
