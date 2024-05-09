/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "sim_guidance.h"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <math.h>
#include <uORB/topics/parameter_update.h>
//#include "waypoints.hpp"

int SIM_GUIDANCE::print_status()
{
	PX4_INFO("Running");
	traj.print_status();

	return 0;
}

int SIM_GUIDANCE::custom_command(int argc, char *argv[])
{

	if (!is_running()) {
		print_usage("not running");
		return 1;
	}



	// additional custom commands can be handled like this:
	for (int i = 0; i < argc; i++)
	{
		if (!strcmp(argv[i], "set_src")) {
			if (argc < i+1)
			{
				PX4_WARN("Please specify a directory, followed by file name");
				print_usage();
				return 0;
			}
			const char *file_string = nullptr;
			if (argc > i+2)
			{
				const char *dir_string = nullptr;
				dir_string = argv[i+1];
				file_string = argv[i+2];
				if (get_instance()->traj.set_src(dir_string, file_string) < 0)
				{
					PX4_WARN("Failed to set new file path for trajectory execution");
					return 0;
				}
			}
			else
			{
				file_string = argv[i+1];
				if (get_instance()->traj.set_src(file_string) < 0)
				{
					PX4_WARN("Failed to set new file path for trajectory execution");
					return 0;
				}
			}

			return 0;
		}
		else if(!strcmp(argv[i], "ls"))
		{
			if (argc < i+2)
			{
				const char* directory_ = get_instance()->traj.file_loader.get_dir();
				//PX4_WARN("Please specify a directory");
				if (get_instance()->traj.file_loader.list_dirs(directory_) < 0)
				{
					PX4_WARN("Failed to list directories");
					return 0;
				}
				if (get_instance()->traj.file_loader.list_files(directory_) < 0)
				{
					PX4_WARN("Failed to list files");
					return 0;
				}
				return 0;
			}
			else
			{
				const char *directory_ = nullptr;
				directory_ = argv[i+1];
				if (get_instance()->traj.file_loader.list_dirs(directory_) < 0)
				{
					PX4_WARN("Failed to list directories");
					return 0;
				}
				if (get_instance()->traj.file_loader.list_files(directory_) < 0)
				{
					PX4_WARN("Failed to list files");
					return 0;
				}
				return 0;
			}

		}
		else if(!strcmp(argv[i], "test"))
		{
			if (argc - 1 > i)
			{
				if (!strcmp(argv[i+1], "solver"))
				{
					//test_solver_codegen();
					return 0;
				}
				else
				{
					PX4_WARN("Uknown test routine. \n\
						Please specify test routine from the list:\n\
						solver");
					return 0;
				}
			}
			else
			{
				PX4_WARN("Please specify test routine from the list:\n\
				solver");
				return 0;
			}
		}
		else continue;
	}


	return print_usage("unknown command");
}


int SIM_GUIDANCE::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("SIM_GUIDANCE",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT - 5,
				      1800,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

SIM_GUIDANCE *SIM_GUIDANCE::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	const char *file_string = nullptr;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			PX4_INFO("p=%i",example_param);
			break;

		case 'f':
			file_string = myoptarg;

			PX4_INFO("f=%s",file_string);
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	SIM_GUIDANCE *instance = new SIM_GUIDANCE(example_param);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

SIM_GUIDANCE::SIM_GUIDANCE(int example_param)
	: ModuleParams(nullptr)
{
}

//#define DEBUG



void SIM_GUIDANCE::run()
{
	// initialize parameters
	parameters_update(true);

	_boot_timestamp = hrt_absolute_time();
	while (!should_exit()) {
		parameters_update(); // update parameters
		update_guidance(); //update everything related to simulink

		px4_usleep(5000);// don't update too frequenty
	}
}

template <typename Type, size_t M>
void assign_1Darray2Vector(matrix::Vector<Type, M> *output_Vec, Type input_1Darray[M])
{
	for (int i = 0; i < M; i++) output_Vec(i) = input_1Darray[i];
	return;
}

void SIM_GUIDANCE::update_guidance(void)
{
	int32_t enable_fl = _param_smg_en.get();


	if (enable_fl > 0)
	{
		#ifdef DEBUG
		PX4_INFO("Updating main loop");
		#endif
		traj.update(enable_fl == 2);

	}


}



void SIM_GUIDANCE::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int SIM_GUIDANCE::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided SIM_GUIDANCE module functionality.

This is a template for a SIM_GUIDANCE module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this SIM_GUIDANCE module.

### Examples
CLI usage example:
$ sim_guidance start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sim_guidance", "custom");
	PRINT_MODULE_USAGE_COMMAND("start");
	//PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_STRING('f', "", nullptr, "Test load directory", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int sim_guidance_main(int argc, char *argv[])
{
	return SIM_GUIDANCE::main(argc, argv);
}
