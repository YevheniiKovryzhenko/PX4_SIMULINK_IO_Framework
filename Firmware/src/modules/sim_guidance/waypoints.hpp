#pragma once

#include <matrix/math.hpp>

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "rt_nonfinite.h"
#include "solver_codegen.h"
#include "solver_codegen_emxAPI.h"
#include "solver_codegen_types.h"

typedef enum
{
        basic_1,
        line_x_1,
        line_xy_1,
        line_xyz_1,
        rrt_test_1,
        circle_1,
        circle_1_slow,
        square_1,
        square_1_slow,
        fig_8_1,
        fig_8_1_slow,
        fig_8_yaw_1,
        fig_8_yaw_1_slow
} trajectory_type_t;

bool solve_preset_traj(trajectory_type_t traj_type,\
        double** wpts, uint16_t wpts_size[2], double* Tf, bool use_time_allocation, bool show_details,\
        double** pp, double** T_out, double* offsets, uint16_t* N_dim, uint16_t* N_segments);

bool solve_trajectory_min_snap(double* wpts_data, uint16_t size[2], double Tf, bool use_time_allocation, bool show_details,\
        double** pp_out, double* T_out, double* offset_out, uint16_t* N_dim_out, uint16_t* N_segments_out);

void test_solver_codegen(void);
