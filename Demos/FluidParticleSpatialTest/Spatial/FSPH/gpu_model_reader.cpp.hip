//
// gpu_model_reader.cpp
// Hybrid_Parallel_SPH
//
// created by ruanjm on 2016/04/15
// Copyright (c) 2016 ruanjm. All rights reserved.
//


#include <hip/hip_runtime.h>
#include "gpu_model_reader.h.hip"
#include <fstream>
#include <iostream>
#include <json/json.h>
#include <json/reader.h>

namespace gpu_model
{

unsigned int readPTXStatisticsFromFile(PTXBlockStatistic *&output, const std::string &func_name, const std::string &file_name)
{
    std::ifstream input(file_name, std::ios::binary);
    if (!input.is_open())
    {
        std::cout << "can not open " << file_name << std::endl;
        return 0;
    }

    unsigned int num_block = 0;

    Json::Reader js_reader;
    Json::Value root;

    if (js_reader.parse(input, root))
    {
        for (int i = 0; i < root.size(); ++i)
        {
            if (func_name == root[i]["function_name"].asString())
            {
                Json::Value blocks = root[i]["block_array"];
                num_block = blocks.size();
                output = new PTXBlockStatistic[blocks.size()];
                for (int j = 0; j < blocks.size(); ++j)
                {
                    PTXBlockStatistic stat;

                    stat.num_insts = blocks[j]["num_insts"].asUInt();
                    stat.num_unknown = blocks[j]["num_unknown"].asUInt();
                    stat.num_sync = blocks[j]["num_sync"].asUInt();
                    stat.num_bra = blocks[j]["num_bra"].asUInt();
                    stat.num_stat[I_ADD_SUB] = blocks[j]["I_ADD_SUB"].asUInt();
                    stat.num_stat[I_MAD_MUL] = blocks[j]["I_MAD_MUL"].asUInt();
                    stat.num_stat[I_DIV_REM] = blocks[j]["I_DIV_REM"].asUInt();
                    stat.num_stat[I_MIN_MAX] = blocks[j]["I_MIN_MAX"].asUInt();
                    stat.num_stat[I_ABS] = blocks[j]["I_ABS"].asUInt();
                    stat.num_stat[I_MUL24] = blocks[j]["I_MUL24"].asUInt();
                    stat.num_stat[I_LOGICAL] = blocks[j]["I_LOGICAL"].asUInt();
                    stat.num_stat[I_SHL_SHR] = blocks[j]["I_SHL_SHR"].asUInt();
                    stat.num_stat[I_SAD] = blocks[j]["I_SAD"].asUInt();
                    stat.num_stat[F_ADD_SUB] = blocks[j]["F_ADD_SUB"].asUInt();
                    stat.num_stat[F_MAD_MUL] = blocks[j]["F_MAD_MUL"].asUInt();
                    stat.num_stat[F_DIV] = blocks[j]["F_DIV"].asUInt();
                    stat.num_stat[F_DIVIDEF] = blocks[j]["F_DIVIDEF"].asUInt();
                    stat.num_stat[F_EXP2] = blocks[j]["F_EXP2"].asUInt();
                    stat.num_stat[F_LOG2] = blocks[j]["F_LOG2"].asUInt();
                    stat.num_stat[F_SIN_COS] = blocks[j]["F_SIN_COS"].asUInt();
                    stat.num_stat[F_SQRT] = blocks[j]["F_SQRT"].asUInt();
                    stat.num_stat[F_RSQRT] = blocks[j]["F_RSQRT"].asUInt();
                    stat.num_stat[F_MIN_MAX] = blocks[j]["F_MIN_MAX"].asUInt();
                    stat.num_stat[F_RCP] = blocks[j]["F_RCP"].asUInt();
                    stat.num_mem[GLOBAL_ACC] = blocks[j]["GLOBAL_ACC"].asUInt();
                    stat.num_mem[SHARED_ACC] = blocks[j]["SHARED_ACC"].asUInt();
                    stat.num_mem[LOCAL_ACC] = blocks[j]["LOCAL_ACC"].asUInt();
                    stat.num_mem[CONST_ACC] = blocks[j]["CONST_ACC"].asUInt();

                    output[j] = stat;
                }

                break;
            }
        }

        // set recommendation
        if (num_block == 7) // SMS
        {
            output[0].recommended_times = 1;
            output[1].recommended_times = 27 / 2;
            output[2].recommended_times = 1;
            output[3].recommended_times = NO_RECOMMENDATION;
            output[4].recommended_times = NO_RECOMMENDATION;
            output[5].recommended_times = NO_RECOMMENDATION;
            output[6].recommended_times = 1;
        }

        if (num_block == 5) // traditional
        {
            output[0].recommended_times = 1;
            output[1].recommended_times = 27;
            output[2].recommended_times = NO_RECOMMENDATION;
            output[3].recommended_times = 27;
            output[4].recommended_times = 1;
        }
    }

    return num_block;
}

void readInstructionLatencyFromFile(InstructionInfo &inst_info, const std::string &file_name)
{
    std::ifstream input(file_name, std::ios::binary);
    if (!input.is_open())
    {
        std::cout << "can not open " << file_name << std::endl;
        return;
    }

    Json::Reader js_reader;
    Json::Value root;

    if (js_reader.parse(input, root))
    {
        inst_info[I_ADD_SUB] = root["I_ADD_SUB"].asFloat();
        inst_info[I_MAD_MUL] = root["I_MAD_MUL"].asFloat();
        inst_info[I_DIV_REM] = root["I_DIV_REM"].asFloat();
        inst_info[I_MIN_MAX] = root["I_MIN_MAX"].asFloat();
        inst_info[I_ABS] = root["I_ABS"].asFloat();
        inst_info[I_MUL24] = root["I_MUL24"].asFloat();
        inst_info[I_LOGICAL] = root["I_LOGICAL"].asFloat();
        inst_info[I_SHL_SHR] = root["I_SHL_SHR"].asFloat();
        inst_info[I_SAD] = root["I_SAD"].asFloat();
        inst_info[F_ADD_SUB] = root["F_ADD_SUB"].asFloat();
        inst_info[F_MAD_MUL] = root["F_MAD_MUL"].asFloat();
        inst_info[F_DIV] = root["F_DIV"].asFloat();
        inst_info[F_DIVIDEF] = root["F_DIVIDEF"].asFloat();
        inst_info[F_EXP2] = root["F_EXP2"].asFloat();
        inst_info[F_LOG2] = root["F_LOG2"].asFloat();
        inst_info[F_SIN_COS] = root["F_SIN_COS"].asFloat();
        inst_info[F_SQRT] = root["F_SQRT"].asFloat();
        inst_info[F_RSQRT] = root["F_RSQRT"].asFloat();
        inst_info[F_MIN_MAX] = root["F_MIN_MAX"].asFloat();
        inst_info[F_RCP] = root["F_RCP"].asFloat();
    }
}

}