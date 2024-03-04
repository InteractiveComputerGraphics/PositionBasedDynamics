//
// sph_parameter.h
// Hybrid_Parallel_SPH
//
// created by ruanjm on 2016/04/05
// Copyright (c) 2016 ruanjm. All rights reserved.
//

#ifndef _SPH_PARAMETER_H
#define _SPH_PARAMETER_H

#include <hip/hip_runtime.h>

namespace sph
{

//sf SPH系统参数
struct SystemParameter
{
    float3      world_size;
    float       cell_size;
    ushort3        grid_size;

    float       kernel;
    float       kernel_2;
    float       mass;




    float       mass1;
    float       mass2;


    float       rest_density1;
    float       rest_density2;

    float       to;
    float       limita;


    float       viscosity1;
    float       viscosity2;



    float3      gravity;
    float       wall_damping;
    float       rest_density;
    float       gas_constant;
    float       viscosity;
    float       time_step;
    float       surface_normal;
    float       surface_coe;

    float       poly6_value;
    float       spiky_value;
    float       visco_value;

    float       grad_poly6;
    float       lplc_poly6;
    float       self_density;
    float       self_lplc_color;

    float3      sim_ratio;
    float3      sim_origin;

    float       bound_interval;
    float3      bound_min;
    float3      bound_max;

	float		spacing_fluid;

	//sf pcisph自加=========================================
	int			pcisph_min_loops;
	int			pcisph_max_loops;
	float		pcisph_max_density_error_allowed;
	float		pcisph_density_error_factor;
};

}

#endif/*_SPH_PARAMETER_H*/
