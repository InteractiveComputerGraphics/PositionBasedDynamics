//
// gpu_model_reader.h
// Hybrid_Parallel_SPH
//
// created by ruanjm on 2016/04/15
// Copyright (c) 2016 ruanjm. All rights reserved.
//

#ifndef _GPU_MODEL_READER_H
#define _GPU_MODEL_READER_H

#include <string>
#include "gpu_model.h.hip"

namespace gpu_model
{

unsigned int readPTXStatisticsFromFile(PTXBlockStatistic *&output, const std::string &func_name, const std::string &file_name);

void readInstructionLatencyFromFile(InstructionInfo &inst_info, const std::string &file_name);

}

#endif/*_GPU_MODEL_READER_H*/