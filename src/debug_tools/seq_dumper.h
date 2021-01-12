#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>

#include <Eigen/Dense>
using namespace Eigen;

using namespace std;
const string SEPARATOR = ", ";

Eigen::Matrix4d _local_to_world_pose(const Eigen::Matrix4d &local_pose);
void create_file(const std::string filename);

void _write_timestamp(std::fstream &file, const double &timestamp, bool separator_at_end=true);
void _write_pose(std::fstream &file, const Eigen::Matrix4d &world_pose, bool separator_at_end=true);
void dump_pose(const string &filename, const double &timestamp, const Eigen::Matrix4d &local_pose);
