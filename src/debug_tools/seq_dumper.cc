
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip> 
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

const std::string SEPARATOR = " ";

Eigen::Matrix4d _local_to_world_pose(const Eigen::Matrix4d &local_pose){
	// Transform the local pose to the world pose
	Eigen::Matrix4d world_pose;
	world_pose.setIdentity();
	world_pose.block<3,3>(0,0) = local_pose.block<3,3>(0,0).transpose();
	world_pose.block<3,1>(0,3) = -world_pose.block<3,3>(0,0) * local_pose.block<3,1>(0,3);
	return world_pose;
}

void _write_timestamp(std::fstream &file, const double &timestamp, bool separator_at_end=true){
	file << std::setprecision(19) << timestamp;
	if (separator_at_end)
		file << SEPARATOR;
}

void _write_pose(std::fstream &file, const Eigen::Matrix4d &world_pose, bool separator_at_end=true){
	Eigen::Quaterniond orientation(world_pose.block<3,3>(0,0));

	file << std::setprecision(6)
			 << std::to_string( world_pose(2,3)) << SEPARATOR
			 << std::to_string(-world_pose(0,3)) << SEPARATOR
			 << std::to_string(-world_pose(1,3)) << SEPARATOR

			 << std::to_string( orientation.z()) << SEPARATOR
			 << std::to_string(-orientation.x()) << SEPARATOR
			 << std::to_string(-orientation.y()) << SEPARATOR
			 << std::to_string( orientation.w());

	if (separator_at_end)
		file << SEPARATOR;
}

void dump_pose(const string &filename, const double &timestamp, const Eigen::Matrix4d &local_pose){

	Eigen::Matrix4d world_pose = _local_to_world_pose(local_pose);

	std::fstream outfile;
	outfile.open(filename, std::fstream::app);

	_write_timestamp(outfile, timestamp, true);
	_write_pose(outfile, world_pose, false);

	outfile << std::endl;
	outfile.close();
}

void create_file(const std::string filename){
	std::fstream outfile;
	outfile.open(filename, std::fstream::out);
	outfile << "# timestamp(ns) px py pz qx qy qz qw" << std::endl;
	outfile.close();
}