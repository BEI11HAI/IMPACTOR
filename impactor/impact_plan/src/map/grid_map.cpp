/************************************************************************
 * Date:        2023.02
 * Author:      Haokun Wang <hwangeh at connect dot ust dot hk>, 
                Aerial Robotics Group <https://uav.ust.hk>, HKUST.
 * E-mail:      hwangeh_at_connect_dot_ust_dot_hk
 * Description: This is the source file for the GridMap class, which is
 *              used to manage the grid map.
 * License:     GNU General Public License <http://www.gnu.org/licenses/>.
 * Project:     IMPACTOR is free software: you can redistribute it and/or 
 *              modify it under the terms of the GNU Lesser General Public 
 *              License as published by the Free Software Foundation, 
 *              either version 3 of the License, or (at your option) any 
 *              later version.
 *              IMPACTOR is distributed in the hope that it will be useful,
 *              but WITHOUT ANY WARRANTY; without even the implied warranty 
 *              of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 *              See the GNU General Public License for more details.
 * Website:     https://github.com/HKUST-Aerial-Robotics/IMPACTOR
 ************************************************************************/

#include "map/grid_map.h"

void GridMap::initMap(ros::NodeHandle& nh) {
	/* get parameter */
	double x_size, y_size, z_size;
	nh.param("grid_map/resolution", mp_.resolution_, -1.0);
	nh.param("grid_map/map_size_x", x_size, -1.0);
	nh.param("grid_map/map_size_y", y_size, -1.0);
	nh.param("grid_map/map_size_z", z_size, -1.0);
	nh.param("grid_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
	nh.param("grid_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
	nh.param("grid_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
	nh.param("grid_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);

	nh.param("grid_map/fx", mp_.fx_, -1.0);
	nh.param("grid_map/fy", mp_.fy_, -1.0);
	nh.param("grid_map/cx", mp_.cx_, -1.0);
	nh.param("grid_map/cy", mp_.cy_, -1.0);

	nh.param("grid_map/use_depth_filter", mp_.use_depth_filter_, true);
	nh.param("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_,
	         -1.0);
	nh.param("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
	nh.param("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
	nh.param("grid_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
	nh.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_,
	         -1.0);
	nh.param("grid_map/skip_pixel", mp_.skip_pixel_, -1);

	nh.param("grid_map/p_hit", mp_.p_hit_, 0.70);
	nh.param("grid_map/p_miss", mp_.p_miss_, 0.35);
	nh.param("grid_map/p_min", mp_.p_min_, 0.12);
	nh.param("grid_map/p_max", mp_.p_max_, 0.97);
	nh.param("grid_map/p_occ", mp_.p_occ_, 0.80);
	nh.param("grid_map/min_ray_length", mp_.min_ray_length_, -0.1);
	nh.param("grid_map/max_ray_length", mp_.max_ray_length_, -0.1);

	nh.param("grid_map/visualization_truncate_height",
	         mp_.visualization_truncate_height_, -0.1);
	nh.param("grid_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);
	nh.param("grid_map/virtual_ceil_yp", mp_.virtual_ceil_yp_, -0.1);
	nh.param("grid_map/virtual_ceil_yn", mp_.virtual_ceil_yn_, -0.1);

	nh.param("grid_map/show_occ_time", mp_.show_occ_time_, false);

	nh.param("grid_map/frame_id", mp_.frame_id_, string("world"));
	nh.param("grid_map/local_map_margin", mp_.local_map_margin_, 1);
	nh.param("grid_map/ground_height", mp_.ground_height_, 0.0);

	nh.param("grid_map/odom_depth_timeout", mp_.odom_depth_timeout_, 1.0);
	// add esdf
	nh.param("grid_map/esdf_slice_height", mp_.esdf_slice_height_, -0.1);
	nh.param("grid_map/show_esdf_time", mp_.show_esdf_time_, false);
	nh.param("grid_map/local_bound_inflate", mp_.local_bound_inflate_, 1.0);

	mp_.local_bound_inflate_ = max(mp_.resolution_, mp_.local_bound_inflate_);
	mp_.resolution_inv_ = 1 / mp_.resolution_;
	mp_.map_origin_ =
	    Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
	mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

	mp_.prob_hit_log_ = logit(mp_.p_hit_);
	mp_.prob_miss_log_ = logit(mp_.p_miss_);
	mp_.clamp_min_log_ = logit(mp_.p_min_);
	mp_.clamp_max_log_ = logit(mp_.p_max_);
	mp_.min_occupancy_log_ = logit(mp_.p_occ_);
	mp_.unknown_flag_ = 0.01;

	cout << "hit: " << mp_.prob_hit_log_ << endl;
	cout << "miss: " << mp_.prob_miss_log_ << endl;
	cout << "min log: " << mp_.clamp_min_log_ << endl;
	cout << "max: " << mp_.clamp_max_log_ << endl;
	cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

	for (int i = 0; i < 3; ++i)
		mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

	mp_.map_min_boundary_ = mp_.map_origin_;
	mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

	if (mp_.virtual_ceil_height_ >= z_size + mp_.ground_height_) {
		mp_.virtual_ceil_height_ =
		    z_size + mp_.ground_height_ - mp_.resolution_;
	}

	// initialize data buffers

	int buffer_size =
	    mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

	md_.occupancy_buffer_ =
	    vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
	md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);

	md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
	md_.count_hit_ = vector<short>(buffer_size, 0);
	md_.flag_rayend_ = vector<char>(buffer_size, -1);
	md_.flag_traverse_ = vector<char>(buffer_size, -1);

	md_.occupancy_buffer_neg_ = vector<char>(buffer_size, 0);
	md_.distance_buffer_ = vector<double>(buffer_size, 10000); // 10000
	md_.distance_buffer_neg_ = vector<double>(buffer_size, 10000);
	md_.distance_buffer_all_ = vector<double>(buffer_size, 10000);
	md_.tmp_buffer1_ = vector<double>(buffer_size, 0);
	md_.tmp_buffer2_ = vector<double>(buffer_size, 0);

	md_.raycast_num_ = 0;

	md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
	md_.proj_points_cnt = 0;

	md_.cam2body_ << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0,
	    0.0, 0.0, 0.0, 0.0, 1.0;

	/* init callback */

	depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(
	    nh, "grid_map/depth", 50));
	extrinsic_sub_ = nh.subscribe<nav_msgs::Odometry>(
	    "/vins_estimator/extrinsic", 10, &GridMap::extrinsicCallback,
	    this); // sub

	pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(
	    nh, "grid_map/pose", 25));

	sync_image_pose_.reset(
	    new message_filters::Synchronizer<SyncPolicyImagePose>(
	        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
	sync_image_pose_->registerCallback(
	    boost::bind(&GridMap::depthPoseCallback, this, _1, _2));

	// use odometry and point cloud
	indep_cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
	    "grid_map/cloud", 10, &GridMap::cloudCallback, this);
	indep_odom_sub_ = nh.subscribe<nav_msgs::Odometry>(
	    "grid_map/odom", 10, &GridMap::odomCallback, this);

	occ_timer_ = nh.createTimer(ros::Duration(0.05),
	                            &GridMap::updateOccupancyCallback, this);
	esdf_timer_ =
	    nh.createTimer(ros::Duration(0.05), &GridMap::updateESDFCallback, this);
	vis_timer_ =
	    nh.createTimer(ros::Duration(0.05), &GridMap::visCallback, this);

	map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);
	map_inf_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
	    "grid_map/occupancy_inflate", 10);
	esdf_pub_ = nh.advertise<sensor_msgs::PointCloud2>("grid_map/esdf", 10);

	md_.occ_need_update_ = false;
	md_.local_updated_ = false;
	md_.has_first_depth_ = false;
	md_.has_odom_ = false;
	md_.has_cloud_ = false;
	md_.image_cnt_ = 0;
	md_.last_occ_update_time_.fromSec(0);
	md_.esdf_need_update_ = false;

	md_.fuse_time_ = 0.0;
	md_.update_num_ = 0;
	md_.max_fuse_time_ = 0.0;

	md_.flag_depth_odom_timeout_ = false;
	md_.flag_use_depth_fusion = false;
}

void GridMap::resetBuffer() {
	Eigen::Vector3d min_pos = mp_.map_min_boundary_;
	Eigen::Vector3d max_pos = mp_.map_max_boundary_;

	resetBuffer(min_pos, max_pos);

	md_.local_bound_min_ = Eigen::Vector3i::Zero();
	md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos) {

	Eigen::Vector3i min_id, max_id;
	posToIndex(min_pos, min_id);
	posToIndex(max_pos, max_id);

	boundIndex(min_id);
	boundIndex(max_id);

	/* reset occ and dist buffer */
	for (int x = min_id(0); x <= max_id(0); ++x)
		for (int y = min_id(1); y <= max_id(1); ++y)
			for (int z = min_id(2); z <= max_id(2); ++z) {
				md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
				md_.distance_buffer_[toAddress(x, y, z)] = 10000;
			}
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ) {
	if (occ != 1 && occ != 0)
		return INVALID_IDX;

	Eigen::Vector3i id;
	posToIndex(pos, id);
	int idx_ctns = toAddress(id);

	md_.count_hit_and_miss_[idx_ctns] += 1;

	if (md_.count_hit_and_miss_[idx_ctns] == 1) {
		md_.cache_voxel_.push(id);
	}

	if (occ == 1)
		md_.count_hit_[idx_ctns] += 1;

	return idx_ctns;
}

void GridMap::projectDepthImage() {
	// md_.proj_points_.clear();
	md_.proj_points_cnt = 0;

	uint16_t* row_ptr;
	// int cols = current_img_.cols, rows = current_img_.rows;
	int cols = md_.depth_image_.cols;
	int rows = md_.depth_image_.rows;
	int skip_pix = mp_.skip_pixel_;

	double depth;

	Eigen::Matrix3d camera_r = md_.camera_r_m_;

	if (!mp_.use_depth_filter_) {
		for (int v = 0; v < rows; v += skip_pix) {
			row_ptr = md_.depth_image_.ptr<uint16_t>(v);

			for (int u = 0; u < cols; u += skip_pix) {

				Eigen::Vector3d proj_pt;
				depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
				proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
				proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
				proj_pt(2) = depth;

				proj_pt = camera_r * proj_pt + md_.camera_pos_;

				if (u == 320 && v == 240)
					std::cout << "depth: " << depth << std::endl;
				md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
			}
		}
	}
	/* use depth filter */
	else {

		if (!md_.has_first_depth_)
			md_.has_first_depth_ = true;
		else {
			Eigen::Vector3d pt_cur, pt_world, pt_reproj;

			Eigen::Matrix3d last_camera_r_inv;
			last_camera_r_inv = md_.last_camera_r_m_.inverse();
			const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

			for (int v = mp_.depth_filter_margin_;
			     v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_) {
				row_ptr = md_.depth_image_.ptr<uint16_t>(v) +
				          mp_.depth_filter_margin_;

				for (int u = mp_.depth_filter_margin_;
				     u < cols - mp_.depth_filter_margin_;
				     u += mp_.skip_pixel_) {

					depth = (*row_ptr) * inv_factor;
					row_ptr = row_ptr + mp_.skip_pixel_;

					if (*row_ptr == 0) {
						depth = mp_.max_ray_length_ + 0.1;
					} else if (depth < mp_.depth_filter_mindist_) {
						continue;
					} else if (depth > mp_.depth_filter_maxdist_) {
						depth = mp_.max_ray_length_ + 0.1;
					}

					// project to world frame
					pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
					pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
					pt_cur(2) = depth;

					pt_world = camera_r * pt_cur + md_.camera_pos_;

					md_.proj_points_[md_.proj_points_cnt++] = pt_world;
				}
			}
		}
	}

	/* maintain camera pose for consistency check */

	md_.last_camera_pos_ = md_.camera_pos_;
	md_.last_camera_r_m_ = md_.camera_r_m_;
	md_.last_depth_image_ = md_.depth_image_;
}

void GridMap::raycastProcess() {
	// if (md_.proj_points_.size() == 0)
	if (md_.proj_points_cnt == 0)
		return;

	ros::Time t1, t2;

	md_.raycast_num_ += 1;

	int vox_idx;
	double length;

	// bounding box of updated region
	double min_x = mp_.map_max_boundary_(0);
	double min_y = mp_.map_max_boundary_(1);
	double min_z = mp_.map_max_boundary_(2);

	double max_x = mp_.map_min_boundary_(0);
	double max_y = mp_.map_min_boundary_(1);
	double max_z = mp_.map_min_boundary_(2);

	RayCaster raycaster;
	Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
	Eigen::Vector3d ray_pt, pt_w;

	for (int i = 0; i < md_.proj_points_cnt; ++i) {
		pt_w = md_.proj_points_[i];

		// set flag for projected point

		if (!isInMap(pt_w)) {
			pt_w = closetPointInMap(pt_w, md_.camera_pos_);

			length = (pt_w - md_.camera_pos_).norm();
			if (length > mp_.max_ray_length_) {
				pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ +
				       md_.camera_pos_;
			}
			vox_idx = setCacheOccupancy(pt_w, 0);
		} else {
			length = (pt_w - md_.camera_pos_).norm();

			if (length > mp_.max_ray_length_) {
				pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ +
				       md_.camera_pos_;
				vox_idx = setCacheOccupancy(pt_w, 0);
			} else {
				vox_idx = setCacheOccupancy(pt_w, 1);
			}
		}

		max_x = max(max_x, pt_w(0));
		max_y = max(max_y, pt_w(1));
		max_z = max(max_z, pt_w(2));

		min_x = min(min_x, pt_w(0));
		min_y = min(min_y, pt_w(1));
		min_z = min(min_z, pt_w(2));

		// raycasting between camera center and point

		if (vox_idx != INVALID_IDX) {
			if (md_.flag_rayend_[vox_idx] == md_.raycast_num_) {
				continue;
			} else {
				md_.flag_rayend_[vox_idx] = md_.raycast_num_;
			}
		}

		raycaster.setInput(pt_w / mp_.resolution_,
		                   md_.camera_pos_ / mp_.resolution_);

		while (raycaster.step(ray_pt)) {
			Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
			length = (tmp - md_.camera_pos_).norm();

			// if (length < mp_.min_ray_length_) break;

			vox_idx = setCacheOccupancy(tmp, 0);

			if (vox_idx != INVALID_IDX) {
				if (md_.flag_traverse_[vox_idx] == md_.raycast_num_) {
					break;
				} else {
					md_.flag_traverse_[vox_idx] = md_.raycast_num_;
				}
			}
		}
	}

	min_x = min(min_x, md_.camera_pos_(0));
	min_y = min(min_y, md_.camera_pos_(1));
	min_z = min(min_z, md_.camera_pos_(2));

	max_x = max(max_x, md_.camera_pos_(0));
	max_y = max(max_y, md_.camera_pos_(1));
	max_z = max(max_z, md_.camera_pos_(2));
	max_z = max(max_z, mp_.ground_height_);

	posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
	posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

	int esdf_inf = ceil(mp_.local_bound_inflate_ / mp_.resolution_);
	md_.local_bound_max_ += esdf_inf * Eigen::Vector3i(1, 1, 0);
	md_.local_bound_min_ -= esdf_inf * Eigen::Vector3i(1, 1, 0);
	boundIndex(md_.local_bound_min_);
	boundIndex(md_.local_bound_max_);

	md_.local_updated_ = true;

	// update occupancy cached in queue
	Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
	Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

	Eigen::Vector3i min_id, max_id;
	posToIndex(local_range_min, min_id);
	posToIndex(local_range_max, max_id);
	boundIndex(min_id);
	boundIndex(max_id);

	// std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

	while (!md_.cache_voxel_.empty()) {

		Eigen::Vector3i idx = md_.cache_voxel_.front();
		int idx_ctns = toAddress(idx);
		md_.cache_voxel_.pop();

		double log_odds_update =
		    md_.count_hit_[idx_ctns] >=
		            md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns]
		        ? mp_.prob_hit_log_
		        : mp_.prob_miss_log_;

		md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

		if (log_odds_update >= 0 &&
		    md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_) {
			continue;
		} else if (log_odds_update <= 0 &&
		           md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_) {
			md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
			continue;
		}

		bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) &&
		                idx(1) >= min_id(1) && idx(1) <= max_id(1) &&
		                idx(2) >= min_id(2) && idx(2) <= max_id(2);
		if (!in_local) {
			md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
		}

		md_.occupancy_buffer_[idx_ctns] =
		    std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update,
		                      mp_.clamp_min_log_),
		             mp_.clamp_max_log_);
	}
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d& pt,
                                          const Eigen::Vector3d& camera_pt) {
	Eigen::Vector3d diff = pt - camera_pt;
	Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
	Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

	double min_t = 1000000;

	for (int i = 0; i < 3; ++i) {
		if (fabs(diff[i]) > 0) {

			double t1 = max_tc[i] / diff[i];
			if (t1 > 0 && t1 < min_t)
				min_t = t1;

			double t2 = min_tc[i] / diff[i];
			if (t2 > 0 && t2 < min_t)
				min_t = t2;
		}
	}

	return camera_pt + (min_t - 1e-3) * diff;
}

void GridMap::clearAndInflateLocalMap() {
	/*clear outside local*/
	const int vec_margin = 5;
	// Eigen::Vector3i min_vec_margin = min_vec -
	// Eigen::Vector3i(vec_margin, vec_margin, vec_margin); Eigen::Vector3i
	// max_vec_margin = max_vec + Eigen::Vector3i(vec_margin, vec_margin,
	// vec_margin);

	Eigen::Vector3i min_cut =
	    md_.local_bound_min_ - Eigen::Vector3i(mp_.local_map_margin_,
	                                           mp_.local_map_margin_,
	                                           mp_.local_map_margin_);
	Eigen::Vector3i max_cut =
	    md_.local_bound_max_ + Eigen::Vector3i(mp_.local_map_margin_,
	                                           mp_.local_map_margin_,
	                                           mp_.local_map_margin_);
	boundIndex(min_cut);
	boundIndex(max_cut);

	Eigen::Vector3i min_cut_m =
	    min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
	Eigen::Vector3i max_cut_m =
	    max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
	boundIndex(min_cut_m);
	boundIndex(max_cut_m);

	// clear data outside the local range

	for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
		for (int y = min_cut_m(1); y <= max_cut_m(1); ++y) {

			for (int z = min_cut_m(2); z < min_cut(2); ++z) {
				int idx = toAddress(x, y, z);
				md_.occupancy_buffer_[idx] =
				    mp_.clamp_min_log_ - mp_.unknown_flag_;
				md_.distance_buffer_all_[idx] = 10000;
			}

			for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z) {
				int idx = toAddress(x, y, z);
				md_.occupancy_buffer_[idx] =
				    mp_.clamp_min_log_ - mp_.unknown_flag_;
				md_.distance_buffer_all_[idx] = 10000;
			}
		}

	for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
		for (int x = min_cut_m(0); x <= max_cut_m(0); ++x) {

			for (int y = min_cut_m(1); y < min_cut(1); ++y) {
				int idx = toAddress(x, y, z);
				md_.occupancy_buffer_[idx] =
				    mp_.clamp_min_log_ - mp_.unknown_flag_;
				md_.distance_buffer_all_[idx] = 10000;
			}

			for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y) {
				int idx = toAddress(x, y, z);
				md_.occupancy_buffer_[idx] =
				    mp_.clamp_min_log_ - mp_.unknown_flag_;
				md_.distance_buffer_all_[idx] = 10000;
			}
		}

	for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
		for (int z = min_cut_m(2); z <= max_cut_m(2); ++z) {

			for (int x = min_cut_m(0); x < min_cut(0); ++x) {
				int idx = toAddress(x, y, z);
				md_.occupancy_buffer_[idx] =
				    mp_.clamp_min_log_ - mp_.unknown_flag_;
				md_.distance_buffer_all_[idx] = 10000;
			}

			for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x) {
				int idx = toAddress(x, y, z);
				md_.occupancy_buffer_[idx] =
				    mp_.clamp_min_log_ - mp_.unknown_flag_;
				md_.distance_buffer_all_[idx] = 10000;
			}
		}

	// inflate occupied voxels to compensate robot size

	int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
	// int inf_step_z = 1;
	vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
	// inf_pts.resize(4 * inf_step + 3);
	Eigen::Vector3i inf_pt;

	// clear outdated data
	for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
		for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
			for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2);
			     ++z) {
				md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
			}

	// inflate obstacles
	for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
		for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
			for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2);
			     ++z) {

				if (md_.occupancy_buffer_[toAddress(x, y, z)] >
				    mp_.min_occupancy_log_) {
					inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

					for (int k = 0; k < (int)inf_pts.size(); ++k) {
						inf_pt = inf_pts[k];
						int idx_inf = toAddress(inf_pt);
						if (idx_inf < 0 ||
						    idx_inf >= mp_.map_voxel_num_(0) *
						                   mp_.map_voxel_num_(1) *
						                   mp_.map_voxel_num_(2)) {
							continue;
						}
						md_.occupancy_buffer_inflate_[idx_inf] = 1;
					}
				}
			}

	// add virtual ceiling to limit flight height
	if (mp_.virtual_ceil_height_ > -0.5) {
		int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) *
		                    mp_.resolution_inv_);

		for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
			for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1);
			     ++y) {
				md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
			}
	}
}

void GridMap::visCallback(const ros::TimerEvent& /*event*/) {
	publishMapInflate(true);
	publishMap();

	publishESDF();
}

void GridMap::updateOccupancyCallback(const ros::TimerEvent& /*event*/) {
	if (md_.last_occ_update_time_.toSec() < 1.0)
		md_.last_occ_update_time_ = ros::Time::now();

	if (!md_.occ_need_update_) {
		if (md_.flag_use_depth_fusion &&
		    (ros::Time::now() - md_.last_occ_update_time_).toSec() >
		        mp_.odom_depth_timeout_) {
			ROS_ERROR("odom or depth lost! ros::Time::now()=%f, "
			          "md_.last_occ_update_time_=%f, "
			          "mp_.odom_depth_timeout_=%f",
			          ros::Time::now().toSec(),
			          md_.last_occ_update_time_.toSec(),
			          mp_.odom_depth_timeout_);
			md_.flag_depth_odom_timeout_ = true;
		}
		return;
	}
	md_.last_occ_update_time_ = ros::Time::now();

	/* update occupancy */
	ros::Time t1, t2;
	t1 = ros::Time::now();

	projectDepthImage();
	raycastProcess();

	if (md_.local_updated_)
		clearAndInflateLocalMap();

	t2 = ros::Time::now();

	md_.fuse_time_ += (t2 - t1).toSec();
	md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

	if (mp_.show_occ_time_)
		ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf",
		         (t2 - t1).toSec(), md_.fuse_time_ / md_.update_num_,
		         md_.max_fuse_time_);

	md_.occ_need_update_ = false;
	if (md_.local_updated_)
		md_.esdf_need_update_ = true;
	md_.local_updated_ = false;
}

void GridMap::updateESDFCallback(const ros::TimerEvent& /*event*/) {
	if (!md_.esdf_need_update_)
		return;

	/* esdf */
	ros::Time t1, t2;
	t1 = ros::Time::now();

	updateESDF3d();

	t2 = ros::Time::now();

	md_.esdf_time_ += (t2 - t1).toSec();
	md_.max_esdf_time_ = max(md_.max_esdf_time_, (t2 - t1).toSec());

	if (mp_.show_esdf_time_)
		ROS_WARN("ESDF: cur t = %lf, avg t = %lf, max t = %lf",
		         (t2 - t1).toSec(), md_.esdf_time_ / md_.update_num_,
		         md_.max_esdf_time_);

	md_.esdf_need_update_ = false;
}

void GridMap::depthPoseCallback(
    const sensor_msgs::ImageConstPtr& img,
    const geometry_msgs::PoseStampedConstPtr& pose) {
	/* get depth image */
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

	if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
		(cv_ptr->image)
		    .convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
	}
	cv_ptr->image.copyTo(md_.depth_image_);

	// std::cout << "depth: " << md_.depth_image_.cols << ", " <<
	// md_.depth_image_.rows << std::endl;

	/* get pose */
	md_.camera_pos_(0) = pose->pose.position.x;
	md_.camera_pos_(1) = pose->pose.position.y;
	md_.camera_pos_(2) = pose->pose.position.z;
	md_.camera_r_m_ =
	    Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
	                       pose->pose.orientation.y, pose->pose.orientation.z)
	        .toRotationMatrix();
	if (isInMap(md_.camera_pos_)) {
		md_.has_odom_ = true;
		md_.update_num_ += 1;
		md_.occ_need_update_ = true;
	} else {
		md_.occ_need_update_ = false;
	}

	md_.flag_use_depth_fusion = true;
}

void GridMap::odomCallback(const nav_msgs::OdometryConstPtr& odom) {
	if (md_.has_first_depth_)
		return;

	md_.camera_pos_(0) = odom->pose.pose.position.x;
	md_.camera_pos_(1) = odom->pose.pose.position.y;
	md_.camera_pos_(2) = odom->pose.pose.position.z;

	md_.has_odom_ = true;
}

void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img) {

	pcl::PointCloud<pcl::PointXYZ> latest_cloud;
	pcl::fromROSMsg(*img, latest_cloud);

	md_.has_cloud_ = true;

	if (!md_.has_odom_) {
		std::cout << "no odom!" << std::endl;
		return;
	}

	if (latest_cloud.points.size() == 0)
		return;

	if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) ||
	    isnan(md_.camera_pos_(2)))
		return;

	this->resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
	                  md_.camera_pos_ + mp_.local_update_range_);

	pcl::PointXYZ pt;
	Eigen::Vector3d p3d, p3d_inf;

	int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
	int inf_step_z = 1;

	double max_x, max_y, max_z, min_x, min_y, min_z;

	min_x = mp_.map_max_boundary_(0);
	min_y = mp_.map_max_boundary_(1);
	min_z = mp_.map_max_boundary_(2);

	max_x = mp_.map_min_boundary_(0);
	max_y = mp_.map_min_boundary_(1);
	max_z = mp_.map_min_boundary_(2);

	for (size_t i = 0; i < latest_cloud.points.size(); ++i) {
		pt = latest_cloud.points[i];
		p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

		/* point inside update range */
		Eigen::Vector3d devi = p3d - md_.camera_pos_;
		Eigen::Vector3i inf_pt;

		if (fabs(devi(0)) < mp_.local_update_range_(0) &&
		    fabs(devi(1)) < mp_.local_update_range_(1) &&
		    fabs(devi(2)) < mp_.local_update_range_(2)) {

			/* inflate the point */
			for (int x = -inf_step; x <= inf_step; ++x)
				for (int y = -inf_step; y <= inf_step; ++y)
					for (int z = -inf_step_z; z <= inf_step_z; ++z) {

						p3d_inf(0) = pt.x + x * mp_.resolution_;
						p3d_inf(1) = pt.y + y * mp_.resolution_;
						p3d_inf(2) = pt.z + z * mp_.resolution_;

						max_x = max(max_x, p3d_inf(0));
						max_y = max(max_y, p3d_inf(1));
						max_z = max(max_z, p3d_inf(2));

						min_x = min(min_x, p3d_inf(0));
						min_y = min(min_y, p3d_inf(1));
						min_z = min(min_z, p3d_inf(2));

						posToIndex(p3d_inf, inf_pt);

						if (!isInMap(inf_pt))
							continue;

						int idx_inf = toAddress(inf_pt);

						md_.occupancy_buffer_inflate_[idx_inf] = 1;
					}
		}
	}

	min_x = min(min_x, md_.camera_pos_(0));
	min_y = min(min_y, md_.camera_pos_(1));
	min_z = min(min_z, md_.camera_pos_(2));

	max_x = max(max_x, md_.camera_pos_(0));
	max_y = max(max_y, md_.camera_pos_(1));
	max_z = max(max_z, md_.camera_pos_(2));

	max_z = max(max_z, mp_.ground_height_);

	posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
	posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

	boundIndex(md_.local_bound_min_);
	boundIndex(md_.local_bound_max_);

	// add virtual ceiling to limit flight height
	if (mp_.virtual_ceil_height_ > -0.5) {
		int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) *
		                    mp_.resolution_inv_);
		for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
			for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1);
			     ++y) {
				md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
			}
	}

	md_.esdf_need_update_ = true;
}

void GridMap::publishMap() {

	if (map_pub_.getNumSubscribers() <= 0)
		return;

	pcl::PointXYZ pt;
	pcl::PointCloud<pcl::PointXYZ> cloud;

	Eigen::Vector3i min_cut = md_.local_bound_min_;
	Eigen::Vector3i max_cut = md_.local_bound_max_;

	int lmm = mp_.local_map_margin_ / 2;
	min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
	max_cut += Eigen::Vector3i(lmm, lmm, lmm);

	boundIndex(min_cut);
	boundIndex(max_cut);

	for (int x = min_cut(0); x <= max_cut(0); ++x)
		for (int y = min_cut(1); y <= max_cut(1); ++y)
			for (int z = min_cut(2); z <= max_cut(2); ++z) {
				if (md_.occupancy_buffer_[toAddress(x, y, z)] <
				    mp_.min_occupancy_log_)
					continue;

				Eigen::Vector3d pos;
				indexToPos(Eigen::Vector3i(x, y, z), pos);
				if (pos(2) > mp_.visualization_truncate_height_)
					continue;

				pt.x = pos(0);
				pt.y = pos(1);
				pt.z = pos(2);
				cloud.push_back(pt);
			}

	cloud.width = cloud.points.size();
	cloud.height = 1;
	cloud.is_dense = true;
	cloud.header.frame_id = mp_.frame_id_;
	sensor_msgs::PointCloud2 cloud_msg;

	pcl::toROSMsg(cloud, cloud_msg);
	map_pub_.publish(cloud_msg);
}

void GridMap::publishMapInflate(bool all_info) {

	if (map_inf_pub_.getNumSubscribers() <= 0)
		return;

	pcl::PointXYZ pt;
	pcl::PointCloud<pcl::PointXYZ> cloud;

	Eigen::Vector3i min_cut = md_.local_bound_min_;
	Eigen::Vector3i max_cut = md_.local_bound_max_;

	if (all_info) {
		int lmm = mp_.local_map_margin_;
		min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
		max_cut += Eigen::Vector3i(lmm, lmm, lmm);
	}

	boundIndex(min_cut);
	boundIndex(max_cut);

	for (int x = min_cut(0); x <= max_cut(0); ++x)
		for (int y = min_cut(1); y <= max_cut(1); ++y)
			for (int z = min_cut(2); z <= max_cut(2); ++z) {
				if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
					continue;

				Eigen::Vector3d pos;
				indexToPos(Eigen::Vector3i(x, y, z), pos);
				if (pos(2) > mp_.visualization_truncate_height_)
					continue;

				pt.x = pos(0);
				pt.y = pos(1);
				pt.z = pos(2);
				cloud.push_back(pt);
			}

	cloud.width = cloud.points.size();
	cloud.height = 1;
	cloud.is_dense = true;
	cloud.header.frame_id = mp_.frame_id_;
	sensor_msgs::PointCloud2 cloud_msg;

	pcl::toROSMsg(cloud, cloud_msg);
	map_inf_pub_.publish(cloud_msg);

	// ROS_INFO("pub map");
}

bool GridMap::odomValid() { return md_.has_odom_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] *
//   mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) {
	ori = mp_.map_origin_, size = mp_.map_size_;
}

void GridMap::extrinsicCallback(const nav_msgs::OdometryConstPtr& odom) {
	Eigen::Quaterniond cam2body_q = Eigen::Quaterniond(
	    odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
	    odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
	Eigen::Matrix3d cam2body_r_m = cam2body_q.toRotationMatrix();
	md_.cam2body_.block<3, 3>(0, 0) = cam2body_r_m;
	md_.cam2body_(0, 3) = odom->pose.pose.position.x;
	md_.cam2body_(1, 3) = odom->pose.pose.position.y;
	md_.cam2body_(2, 3) = odom->pose.pose.position.z;
	md_.cam2body_(3, 3) = 1.0;
}

void GridMap::depthOdomCallback(const sensor_msgs::ImageConstPtr& img,
                                const nav_msgs::OdometryConstPtr& odom) {
	/* get pose */
	Eigen::Quaterniond body_q = Eigen::Quaterniond(
	    odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
	    odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
	Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
	Eigen::Matrix4d body2world;
	body2world.block<3, 3>(0, 0) = body_r_m;
	body2world(0, 3) = odom->pose.pose.position.x;
	body2world(1, 3) = odom->pose.pose.position.y;
	body2world(2, 3) = odom->pose.pose.position.z;
	body2world(3, 3) = 1.0;

	Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
	md_.camera_pos_(0) = cam_T(0, 3);
	md_.camera_pos_(1) = cam_T(1, 3);
	md_.camera_pos_(2) = cam_T(2, 3);
	md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);

	/* get depth image */
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
	if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
		(cv_ptr->image)
		    .convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
	}
	cv_ptr->image.copyTo(md_.depth_image_);

	md_.occ_need_update_ = true;
	md_.flag_use_depth_fusion = true;
}

template <typename F_get_val, typename F_set_val>
void GridMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start,
                       int end, int dim) {
	int v[mp_.map_voxel_num_(dim)];
	double z[mp_.map_voxel_num_(dim) + 1];

	int k = start;
	v[start] = start;
	z[start] = -std::numeric_limits<double>::max();
	z[start + 1] = std::numeric_limits<double>::max();

	for (int q = start + 1; q <= end; q++) {
		k++;
		double s;

		do {
			k--;
			s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) /
			    (2 * q - 2 * v[k]);
		} while (s <= z[k]);

		k++;

		v[k] = q;
		z[k] = s;
		z[k + 1] = std::numeric_limits<double>::max();
	}

	k = start;

	for (int q = start; q <= end; q++) {
		while (z[k + 1] < q)
			k++;
		double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
		f_set_val(q, val);
	}
}

void GridMap::updateESDF3d() {
	Eigen::Vector3i min_esdf = md_.local_bound_min_;
	Eigen::Vector3i max_esdf = md_.local_bound_max_;

	/* ========== compute positive DT ========== */
	for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
		for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
			fillESDF(
			    [&](int z) {
				    return md_.occupancy_buffer_inflate_[toAddress(x, y, z)] ==
				                   1
				               ? 0
				               : std::numeric_limits<double>::max();
			    },
			    [&](int z, double val) {
				    md_.tmp_buffer1_[toAddress(x, y, z)] = val;
			    },
			    min_esdf[2], max_esdf[2], 2);
		}
	}

	for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
		for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
			fillESDF(
			    [&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
			    [&](int y, double val) {
				    md_.tmp_buffer2_[toAddress(x, y, z)] = val;
			    },
			    min_esdf[1], max_esdf[1], 1);
		}
	}

	for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
		for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
			fillESDF(
			    [&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
			    [&](int x, double val) {
				    md_.distance_buffer_[toAddress(x, y, z)] =
				        mp_.resolution_ * std::sqrt(val);
				    //  min(mp_.resolution_ * std::sqrt(val),
				    //      distance_buffer_[toAddress(x, y,
				    //      z)]);
			    },
			    min_esdf[0], max_esdf[0], 0);
		}
	}

	/* ========== compute negative distance ========== */
	for (int x = min_esdf(0); x <= max_esdf(0); ++x)
		for (int y = min_esdf(1); y <= max_esdf(1); ++y)
			for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

				int idx = toAddress(x, y, z);
				if (md_.occupancy_buffer_inflate_[idx] == 0) {
					md_.occupancy_buffer_neg_[idx] = 1;
				} else if (md_.occupancy_buffer_inflate_[idx] == 1) {
					md_.occupancy_buffer_neg_[idx] = 0;
				} else {
					ROS_ERROR("what?");
				}
			}

	md_.tmp_buffer1_.clear();
	md_.tmp_buffer2_.clear();

	for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
		for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
			fillESDF(
			    [&](int z) {
				    return md_.occupancy_buffer_neg_[x * mp_.map_voxel_num_(1) *
				                                         mp_.map_voxel_num_(2) +
				                                     y * mp_.map_voxel_num_(2) +
				                                     z] == 1
				               ? 0
				               : std::numeric_limits<double>::max();
			    },
			    [&](int z, double val) {
				    md_.tmp_buffer1_[toAddress(x, y, z)] = val;
			    },
			    min_esdf[2], max_esdf[2], 2);
		}
	}

	for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
		for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
			fillESDF(
			    [&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
			    [&](int y, double val) {
				    md_.tmp_buffer2_[toAddress(x, y, z)] = val;
			    },
			    min_esdf[1], max_esdf[1], 1);
		}
	}

	for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
		for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
			fillESDF(
			    [&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
			    [&](int x, double val) {
				    md_.distance_buffer_neg_[toAddress(x, y, z)] =
				        mp_.resolution_ * std::sqrt(val);
			    },
			    min_esdf[0], max_esdf[0], 0);
		}
	}

	/* ========== combine pos and neg DT ========== */
	for (int x = min_esdf(0); x <= max_esdf(0); ++x)
		for (int y = min_esdf(1); y <= max_esdf(1); ++y)
			for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

				int idx = toAddress(x, y, z);
				md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];

				if (md_.distance_buffer_neg_[idx] > 0.0)
					md_.distance_buffer_all_[idx] +=
					    (-md_.distance_buffer_neg_[idx] + mp_.resolution_);
			}
}

void GridMap::publishESDF() {
	if (esdf_pub_.getNumSubscribers() <= 0)
		return;
	double dist;
	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::PointXYZI pt;

	const double min_dist = -3.0;
	const double max_dist = 3.0;

	// Eigen::Vector3i min_cut = local_bound_min_ -
	//     Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_,
	//     mp_.local_map_margin_);
	// Eigen::Vector3i max_cut = local_bound_max_ +
	//     Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_,
	//     mp_.local_map_margin_);
	Eigen::Vector3i min_cut = md_.local_bound_min_;
	Eigen::Vector3i max_cut = md_.local_bound_max_;
	boundIndex(min_cut);
	boundIndex(max_cut);

	for (int x = min_cut(0); x <= max_cut(0); ++x)
		for (int y = min_cut(1); y <= max_cut(1); ++y) {

			Eigen::Vector3d pos;
			indexToPos(Eigen::Vector3i(x, y, -0.5), pos);
			pos(2) = mp_.esdf_slice_height_;

			dist = getDistance(pos);
			dist = min(dist, max_dist);
			dist = max(dist, min_dist);

			pt.x = pos(0);
			pt.y = pos(1);
			pt.z = pos(2);
			pt.intensity = (dist - min_dist) / (max_dist - min_dist);
			cloud.push_back(pt);
		}

	cloud.width = cloud.points.size();
	cloud.height = 1;
	cloud.is_dense = true;
	cloud.header.frame_id = mp_.frame_id_;
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(cloud, cloud_msg);

	esdf_pub_.publish(cloud_msg);
}

/* use for ESDF API*/

void GridMap::getSurroundPts(const Eigen::Vector3d& pos,
                             Eigen::Vector3d pts[2][2][2],
                             Eigen::Vector3d& diff) {
	/* interpolation position */
	Eigen::Vector3d pos_m =
	    pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();
	Eigen::Vector3i idx;
	Eigen::Vector3d idx_pos;

	posToIndex(pos_m, idx);
	indexToPos(idx, idx_pos);
	diff = (pos - idx_pos) * mp_.resolution_inv_;

	for (int x = 0; x < 2; x++) {
		for (int y = 0; y < 2; y++) {
			for (int z = 0; z < 2; z++) {
				Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
				Eigen::Vector3d current_pos;
				indexToPos(current_idx, current_pos);
				pts[x][y][z] = current_pos;
			}
		}
	}
}

void GridMap::getSurroundDistance(Eigen::Vector3d pts[2][2][2],
                                  double dists[2][2][2]) {
	for (int x = 0; x < 2; x++) {
		for (int y = 0; y < 2; y++) {
			for (int z = 0; z < 2; z++) {
				dists[x][y][z] = getDistance(pts[x][y][z]);
			}
		}
	}
}

pair<double, Eigen::Vector3d>
GridMap::interpolateTrilinear(double values[2][2][2],
                              const Eigen::Vector3d& diff, double& value,
                              Eigen::Vector3d& grad) {
	// trilinear interpolation
	double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0];
	double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1];
	double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0];
	double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1];
	double v0 = (1 - diff(1)) * v00 + diff(1) * v10;
	double v1 = (1 - diff(1)) * v01 + diff(1) * v11;

	value = (1 - diff(2)) * v0 + diff(2) * v1;

	grad[2] = (v1 - v0) * mp_.resolution_inv_;
	grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) *
	          mp_.resolution_inv_;
	grad[0] =
	    (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
	grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
	grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
	grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
	grad[0] *= mp_.resolution_inv_;

	return std::make_pair(value, grad);
}

void GridMap::interpolateTrilinearEDT(double values[2][2][2],
                                      const Eigen::Vector3d& diff,
                                      double& value) {
	// trilinear interpolation
	double v00 =
	    (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0]; // b
	double v01 =
	    (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1]; // d
	double v10 =
	    (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0]; // a
	double v11 =
	    (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1]; // c
	double v0 = (1 - diff(1)) * v00 + diff(1) * v10;                 // e
	double v1 = (1 - diff(1)) * v01 + diff(1) * v11;                 // f

	value = (1 - diff(2)) * v0 + diff(2) * v1;
}

void GridMap::interpolateTrilinearFirstGrad(double values[2][2][2],
                                            const Eigen::Vector3d& diff,
                                            Eigen::Vector3d& grad) {
	// trilinear interpolation
	double v00 =
	    (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0]; // b
	double v01 =
	    (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1]; // d
	double v10 =
	    (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0]; // a
	double v11 =
	    (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1]; // c
	double v0 = (1 - diff(1)) * v00 + diff(1) * v10;                 // e
	double v1 = (1 - diff(1)) * v01 + diff(1) * v11;                 // f

	grad[2] = (v1 - v0) * mp_.resolution_inv_;
	grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) *
	          mp_.resolution_inv_;
	grad[0] =
	    (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
	grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
	grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
	grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
	grad[0] *= mp_.resolution_inv_;
}

void GridMap::evaluateEDT(const Eigen::Vector3d& pos, double& dist) {
	Eigen::Vector3d diff;
	Eigen::Vector3d sur_pts[2][2][2];
	getSurroundPts(pos, sur_pts, diff);

	double dists[2][2][2];
	getSurroundDistance(sur_pts, dists);

	interpolateTrilinearEDT(dists, diff, dist);
}

void GridMap::evaluateFirstGrad(const Eigen::Vector3d& pos,
                                Eigen::Vector3d& grad) {
	Eigen::Vector3d diff;
	Eigen::Vector3d sur_pts[2][2][2];
	getSurroundPts(pos, sur_pts, diff);

	double dists[2][2][2];
	getSurroundDistance(sur_pts, dists);

	interpolateTrilinearFirstGrad(dists, diff, grad);
}

pair<double, Eigen::Vector3d>
GridMap::evaluateEDTWithGrad(const Eigen::Vector3d& pos, double& dist,
                             Eigen::Vector3d& grad) {
	Eigen::Vector3d diff;
	Eigen::Vector3d sur_pts[2][2][2];
	getSurroundPts(pos, sur_pts, diff);
	double dists[2][2][2];
	getSurroundDistance(sur_pts, dists);

	return interpolateTrilinear(dists, diff, dist, grad);
}