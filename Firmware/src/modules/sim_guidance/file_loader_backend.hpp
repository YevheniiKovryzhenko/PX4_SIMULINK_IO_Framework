#pragma once

#include <matrix/math.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <matrix/math.hpp>

typedef struct traj_file_header_t
{
	uint8_t n_coeffs;
	uint8_t n_int;
	uint8_t n_dofs;
}__attribute__((packed)) traj_file_header_t;

typedef struct traj_file_data_t
{
	uint8_t i_int;
	uint8_t i_dof;
	float t_int;
	float coefs[10];
} __attribute__((packed)) traj_file_data_t;

class file_loader_backend
{
private:
	char file_name[256] {};
	char directory[256] = "/fs/microsd/trajectories/";
	int _fd = -1;
	int open_file(void);

public:
	int set_src(const char* _file, const char* _dir);

	int read_header(traj_file_header_t& traj_header);
	int read_data(traj_file_data_t& traj_data);

	int read_dummy_header(traj_file_header_t& traj_header);
	int read_dummy_data(traj_file_data_t& traj_data, int i_int, int i_dof);

	int write_header(traj_file_header_t& traj_header);
	int write_data(traj_file_data_t& traj_data);

	int close_file(void);

	int list_dirs(const char* location);
	int list_files(const char* location);
	int list_abs_path(const char* location);
	const char* get_dir(void);
	const char* get_file(void);

	file_loader_backend();
	~file_loader_backend();
};
