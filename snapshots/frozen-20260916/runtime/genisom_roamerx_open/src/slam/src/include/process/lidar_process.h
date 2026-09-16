#pragma once
#include "common.h"
using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

enum LID_TYPE
{
    AVIA = 1,
    VELO16,
    OUST64,
    MID360
};  //{1, 2, 3}
enum TIME_UNIT
{
    SEC = 0,
    MS  = 1,
    US  = 2,
    NS  = 3
};
enum Feature
{
    Nor,
    Poss_Plane,
    Real_Plane,
    Edge_Jump,
    Edge_Plane,
    Wire,
    ZeroPoint
};
enum Surround
{
    Prev,
    Next
};
enum E_jump
{
    Nr_nor,
    Nr_zero,
    Nr_180,
    Nr_inf,
    Nr_blind
};

struct orgtype
{
    double  range;
    double  dista;
    double  angle[2];
    double  intersect;
    E_jump  edj[2];
    Feature ftype;
    orgtype()
    {
        range     = 0;
        edj[Prev] = Nr_nor;
        edj[Next] = Nr_nor;
        ftype     = Nor;
        intersect = 2;
    }
};

namespace livox_pcl
{
    /**
     * @struct Point
     * @brief Represents a point in a point cloud with additional attributes.
     */
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;                 // Adds x, y, z, and padding fields.
        float        intensity;          ///< Intensity of the laser return.
        double       timestamp;          ///< Timestamp of the point capture.
        std::uint8_t line;               ///< Laser line index.
        std::uint8_t tag;                ///< User-defined tag for categorization.
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Ensures proper alignment for Eigen types.
    };
}  // namespace livox_pcl

/**
 * @brief Registers the `livox_pcl::Point` structure with PCL.
 */
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_pcl::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                                        double, timestamp, timestamp)(std::uint8_t, line, line)(std::uint8_t, tag, tag))


class Preprocess
{
public:
    //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Preprocess();
    ~Preprocess();

    void process(const pcl::PointCloud<livox_pcl::Point>& msg, robot::slam::CloudPtr& pcl_out);
    void set(bool feat_en, int lid_type, double bld, int pfilt_num);

    robot::slam::PointCloudType pl_full, pl_corn, pl_surf;
    robot::slam::PointCloudType pl_buff[128];  // maximum 128 line lidar
    vector<orgtype>             typess[128];   // maximum 128 line lidar
    float                       time_unit_scale;
    int                         lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;
    double                      blind;
    bool                        feature_enabled, given_offset_time;
    // ros::Publisher pub_full, pub_surf, pub_corn;

private:
    void avia_handler(const pcl::PointCloud<livox_pcl::Point>& msg);
    void give_feature(robot::slam::PointCloudType& pl, vector<orgtype>& types);
    int  plane_judge(const robot::slam::PointCloudType& pl, vector<orgtype>& types, uint i, uint& i_nex, Eigen::Vector3d& curr_direct);
    bool small_plane(const robot::slam::PointCloudType& pl, vector<orgtype>& types, uint i_cur, uint& i_nex, Eigen::Vector3d& curr_direct);
    bool edge_jump_judge(const robot::slam::PointCloudType& pl, vector<orgtype>& types, uint i, Surround nor_dir);

    int    group_size;
    double disA, disB, inf_bound;
    double limit_maxmid, limit_midmin, limit_maxmin;
    double p2l_ratio;
    double jump_up_limit, jump_down_limit;
    double cos160;
    double edgea, edgeb;
    double smallp_intersect, smallp_ratio;
    double vx, vy, vz;
};
