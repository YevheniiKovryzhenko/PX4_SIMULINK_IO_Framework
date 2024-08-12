/****************************************************************************
//          Auburn University Aerospace Engineering Department
//             Aero-Astro Computational and Experimental lab
//
//     Copyright (C) 2024  Yevhenii Kovryzhenko
//
//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the
//     GNU AFFERO GENERAL PUBLIC LICENSE Version 3
//     along with this program.  If not, see <https://www.gnu.org/licenses/>
//
****************************************************************************/

#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>

#include "file_loader_backend.hpp"

file_loader_backend::file_loader_backend()
{

}

file_loader_backend::~file_loader_backend()
{
}

int file_loader_backend::list_dirs(const char* location)
{
	//const char* location = "."; // Replace with the path you want to list directories in
	DIR* dir = opendir(location);

	if (dir == NULL) {
		PX4_WARN("Failed to open directory");
		return -1;
	}

	struct dirent* entry;

	while ((entry = readdir(dir)) != NULL) {
		if (entry->d_type == DT_DIR) {
			if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0) {
				PX4_INFO("%s/", entry->d_name);
			}
		}
	}

	closedir(dir);

	return 0;
}

int file_loader_backend::list_files(const char* location)
{
	//const char* location = "."; // Replace with the path you want to list files in
	DIR* dir = opendir(location);

	if (dir == NULL) {
		PX4_WARN("Failed to open directory");
		return -1;
	}

	struct dirent* entry;

	while ((entry = readdir(dir)) != NULL) {
		if (entry->d_type == DT_REG) {
			PX4_INFO("%s", entry->d_name);
		}
	}

	closedir(dir);

	return 0;
}

void concatenatePaths(char* result, const char* directory, const char* filename) {
    // Check if the directory path ends with a slash
    int dir_length = strlen(directory);
    if (dir_length > 0 && directory[dir_length - 1] != '/') {
        // Append a slash to the directory if it's missing
        snprintf(result, 256, "%s/%s", directory, filename);
    } else {
        // No need to add an extra slash
        snprintf(result, 256, "%s%s", directory, filename);
    }
}

int file_loader_backend::list_abs_path(const char* location)
{
	char* resolved_path;
	resolved_path = realpath(location, NULL);
	if (resolved_path == NULL) {
		PX4_WARN("Failed to get the absolute path");
		return -1;
	}
	PX4_INFO("%s", resolved_path);
	free(resolved_path);
	return 0;
}


int file_loader_backend::set_src(const char* _file, const char* _dir)
{
	// Check if the directory exists, and if not, create it
	if (access(_dir, F_OK) == -1) {
		PX4_WARN("Directory does not exist\n");
		return -1;
		/*
		if (mkdir(_dir, S_IRWXU | S_IRWXG | S_IRWXO) != 0) {
			if (errno != EEXIST) {
				PX4_WARN("Failed to create the directory: %d", errno);
				return -1;
			} else {
				PX4_INFO("Directory created: %s", _dir);
			}
		}
		*/
	}

	// Create a buffer to hold the full file path
	char filepath[256]; // Adjust the size as needed
	concatenatePaths(filepath,  _dir, _file);

	// Check if the file exists in the directory
	if (access(filepath, F_OK) == -1) {
		PX4_WARN("File %s not found in the directory.", _file);
		return -1;
	}

	close_file(); //make sure old file is closed

	/* store file name and dirrectory */
	strncpy(file_name, filepath, sizeof(filepath) - 1);
	strncpy(directory, _dir, sizeof(_dir) - 1);

	/* enforce null termination */
	file_name[sizeof(file_name) - 1] = '\0';
	directory[sizeof(directory) - 1] = '\0';

	PX4_INFO("Configured new trajectory file location.");
	list_abs_path(file_name);
	return 0;
}

//#define DEBUG

int file_loader_backend::read_dummy_header(traj_file_header_t& header)
{
	header.n_coeffs = 10;
	header.n_int = 2;
	header.n_dofs = 4;

	return 0;
}

int file_loader_backend::read_dummy_data(traj_file_data_t& data, int i_int, int i_dof)
{

        static const float tof_int_raw[] = {2.107510627081439, 2.892489372918561};
	static const float x0_coefs_raw[] = {0.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};
	static const float y0_coefs_raw[] = {0.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};
	static const float z0_coefs_raw[] = {0.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};
	static const float yaw0_coefs_raw[] = {0.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};

	static const float x1_coefs_raw[] = {1,3.667080492742769,2.117588692858374,-5.141681182811569,-3.594994733263419,16.288952666450880,-37.844351520577504,53.726887291011200,-36.411234714857710,9.191753008446979};
	static const float y1_coefs_raw[] = {1,3.667080492742769,2.117588692858374,-5.141681182811569,-3.594994733263419,16.288952666450880,-37.844351520577504,53.726887291011200,-36.411234714857710,9.191753008446979};
	static const float z1_coefs_raw[] = {1,3.667080492742769,2.117588692858374,-5.141681182811569,-3.594994733263419,16.288952666450880,-37.844351520577504,53.726887291011200,-36.411234714857710,9.191753008446979};
	static const float yaw1_coefs_raw[] = {1,3.667080492742769,2.117588692858374,-5.141681182811569,-3.594994733263419,16.288952666450880,-37.844351520577504,53.726887291011200,-36.411234714857710,9.191753008446979};

	data.i_dof = i_dof;
	data.i_int = i_int;
	data.t_int = tof_int_raw[i_int];
	switch (i_int)
	{
	case 0:
		switch (i_dof)
		{
		case 0:
			for (int i = 0; i < 10; i++) data.coefs[i] = x0_coefs_raw[i];
			break;
		case 1:
			for (int i = 0; i < 10; i++) data.coefs[i] = y0_coefs_raw[i];
			break;
		case 2:
			for (int i = 0; i < 10; i++) data.coefs[i] = z0_coefs_raw[i];
			break;

		default:
			for (int i = 0; i < 10; i++) data.coefs[i] = yaw0_coefs_raw[i];
			break;
		}
		break;

	default:
		switch (i_dof)
		{
		case 0:
			for (int i = 0; i < 10; i++) data.coefs[i] = x1_coefs_raw[i];
			break;
		case 1:
			for (int i = 0; i < 10; i++) data.coefs[i] = y1_coefs_raw[i];
			break;
		case 2:
			for (int i = 0; i < 10; i++) data.coefs[i] = z1_coefs_raw[i];
			break;

		default:
			for (int i = 0; i < 10; i++) data.coefs[i] = yaw1_coefs_raw[i];
			break;
		}
		break;
	}



	return 0;
}

int file_loader_backend::read_header(traj_file_header_t& header)
{
	if (open_file() < 0)
	{
		PX4_ERR("Failed to open file");
		return -1;
	}

        if (px4_read(_fd, &header, sizeof(traj_file_header_t)) < 0)
	{
            PX4_ERR("Failed to read header from %d, because %d", _fd, errno);
            return -1;
        }

	#ifdef DEBUG
		PX4_INFO("Read data:");
		printf("n_int=%u, n_dofs=%u, n_coeffs=%u\n",header.n_int, header.n_dofs, header.n_coeffs);
	#endif

	return 0;
}

int file_loader_backend::write_header(traj_file_header_t& header)
{
	if (open_file() < 0)
	{
		PX4_ERR("Failed to open file");
		return -1;
	}

        if (px4_write(_fd, &header, sizeof(traj_file_header_t)) < 0)
	{
            PX4_ERR("Failed to write header to %d, because %d", _fd, errno);
            return -1;
        }
	return 0;
}



int file_loader_backend::read_data(traj_file_data_t& data)
{
	if (open_file() < 0)
	{
		PX4_ERR("Failed to open file");
		return -1;
	}

        if (px4_read(_fd, &data, sizeof(traj_file_data_t)) < 0)
	{
            PX4_ERR("Failed to read data from %d, because %d", _fd, errno);
            return -1;
        }

	#ifdef DEBUG
		PX4_INFO("Read data:");
		PX4_INFO("i_int=%u, i_dof=%u, t_int=%f",data.i_int, data.i_dof, (double)data.t_int);
		for(int i = 0; i < 10; i++) printf(", coeff[%i]=%f", i, (double)data.coefs[i]);
		printf("\n");
	#endif
	return 0;
}

int file_loader_backend::write_data(traj_file_data_t& data)
{
	if (open_file() < 0)
	{
		PX4_ERR("Failed to open file");
		return -1;
	}

        if (px4_read(_fd, &data, sizeof(traj_file_data_t)) < 0)
	{
            PX4_ERR("Failed to write data to %d, because %d", _fd, errno);
            return -1;
        }
	return 0;
}

int file_loader_backend::open_file(void)
{
	if (_fd < 0)
	{
		_fd = px4_open(file_name, O_RDONLY);
		if (_fd < 0)
		{
			PX4_ERR("Can't open file, %d", errno);
			return -1;
		}
		PX4_INFO("Opened file!");
	}

	return 0;
}

const char* file_loader_backend::get_dir(void)
{
	return directory;
}
const char* file_loader_backend::get_file(void)
{
	return file_name;
}

int file_loader_backend::close_file(void)
{
	if (_fd > -1)
	{
		px4_close(_fd);
		_fd = -1;
	}
	return 0;
}


