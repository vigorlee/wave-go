"""Compile the real LIO deskew loop without starting ROS or a simulator."""
import json
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'genisom_roamerx_open/src/slam/src/src/process/imu_process.cpp'

# Consistent harmonic IMU poses exercise both a normal scan and the measured
# 4.335889-second source gap. This is an isolated software test, not map data.
HARNESS = r'''
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <vector>
#include <iostream>
#include <cmath>
#include <cstdint>
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]
namespace robot { namespace slam { using Vec3d=Eigen::Vector3d; using Mat3d=Eigen::Matrix3d; } }
using Vec=Eigen::Vector3d;
using Mat=Eigen::Matrix3d;
Mat Exp(const Vec& w, double dt) {
  const double n=w.norm();
  return n<1e-12 ? Mat::Identity() : Eigen::AngleAxisd(n*dt,w/n).toRotationMatrix();
}
struct Pose { double offset_time; std::array<double,9> rot; std::array<double,3> vel,pos,acc,gyr; };
struct Point { double x,y,z,curvature; };
struct Cloud { std::vector<Point> points; };
struct State { Eigen::Quaterniond rot,offset_R_L_I; Vec pos,offset_T_L_I; };
void deskew(Cloud& pcl_out, const std::vector<Pose>& IMUpose,
            const State& imu_state, std::vector<int>& touch_count) {
  double dt=0.; Mat R_imu; Vec vel_imu,pos_imu,acc_imu,angvel_avr;
  /* SOURCE_LOOP */
}
std::array<double,3> array(const Vec& v) { return {v.x(),v.y(),v.z()}; }
Pose pose(double t) {
  const double f=6.283185307179586;
  const double yaw=.15*std::sin(.7*f*t);
  const Mat r=Eigen::AngleAxisd(yaw,Vec::UnitZ()).toRotationMatrix();
  Pose p; p.offset_time=t;
  for(int i=0;i<3;++i) for(int j=0;j<3;++j) p.rot[3*i+j]=r(i,j);
  p.pos={.10*std::sin(f*t),.08*std::cos(.8*f*t),.06*std::sin(.6*f*t)};
  p.vel={.10*f*std::cos(f*t),-.08*.8*f*std::sin(.8*f*t),.06*.6*f*std::cos(.6*f*t)};
  p.acc={-.10*f*f*std::sin(f*t),-.08*.64*f*f*std::cos(.8*f*t),-.06*.36*f*f*std::sin(.6*f*t)};
  p.gyr={0.,0.,.15*.7*f*std::cos(.7*f*t)};
  return p;
}
void run(double gap, bool mixed, bool first) {
  std::vector<Pose> poses{pose(0.)};
  const int intervals=std::max(2,int(std::ceil(gap/.005)));
  for(int i=0;i<=intervals;++i) poses.push_back(pose(-gap+gap*i/intervals));
  const Pose end=pose(0.);
  State state; state.rot=Eigen::Quaterniond::Identity();state.pos=Vec(end.pos.data());
  state.offset_R_L_I=Eigen::Quaterniond(Eigen::AngleAxisd(.3490658503988659,Vec::UnitY()));
  state.offset_T_L_I=Vec(.15568,-.02329,.13366);
  Cloud cloud;
  for(int i=0;i<200;++i) {
    const double a=.123*i;
    cloud.points.push_back({9.*std::cos(a),8.*std::sin(a),5.*std::sin(.71*a),mixed?gap*1000.*i/199.:0.});
  }
  std::vector<int> touches(cloud.points.size(),0);
  deskew(cloud,poses,state,touches);
  Vec lo=Vec::Constant(1e300),hi=Vec::Constant(-1e300);double max_norm=0.;
  for(const auto& p:cloud.points) { Vec v(p.x,p.y,p.z);lo=lo.cwiseMin(v);hi=hi.cwiseMax(v);max_norm=std::max(max_norm,v.norm()); }
  std::int64_t cells=1;for(int i=0;i<3;++i) cells*=std::int64_t((hi[i]-lo[i])/.1)+1;
  if(!first) std::cout<<",";
  std::cout<<"{\"gap\":"<<gap<<",\"mixed_offsets\":"<<(mixed?"true":"false")
           <<",\"first_touches\":"<<touches.front()<<",\"last_touches\":"<<touches.back()
           <<",\"max_touches\":"<<*std::max_element(touches.begin(),touches.end())
           <<",\"max_point_norm\":"<<max_norm<<",\"voxel_bbox_index_product\":"<<cells<<"}";
}
int main() { std::cout<<"[";run(.1,false,true);run(4.3358891,false,false);run(.1,true,false);std::cout<<"]\n"; }
'''


def evaluate(source=SOURCE):
    text = source.read_text()
    begin = text.index('    auto it_pcl = pcl_out.points.end() - 1;')
    end = text.index('\n}\n\nvoid ImuProcess::Process', begin)
    loop = text[begin:end]
    # Count operations while preserving the production loop and transforms.
    loop = loop.replace('            it_pcl->x = P_compensate(0);',
                        '            ++touch_count[std::distance(pcl_out.points.begin(), it_pcl)];\n'
                        '            it_pcl->x = P_compensate(0);')
    assert '++touch_count' in loop
    with tempfile.TemporaryDirectory(prefix='lio_deskew_boundary_') as directory:
        folder = Path(directory)
        cpp = folder / 'loop.cpp'
        exe = folder / 'loop'
        cpp.write_text(HARNESS.replace('/* SOURCE_LOOP */', loop))
        subprocess.run(['g++', '-std=c++17', '-O2', '-I/usr/include/eigen3', str(cpp), '-o', str(exe)],
                       check=True, capture_output=True, text=True)
        output = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
        return json.loads(output.stdout)


class LioDeskewBoundaryTest(unittest.TestCase):
    def test_each_point_is_deskewed_once_including_first_zero_offset(self):
        for case in evaluate():
            with self.subTest(gap=case['gap'], mixed=case['mixed_offsets']):
                self.assertEqual(case['first_touches'], 1)
                self.assertEqual(case['last_touches'], 1)
                self.assertEqual(case['max_touches'], 1)
                self.assertLess(case['voxel_bbox_index_product'], 2**31 - 1)
                self.assertLess(case['max_point_norm'], 30.)


if __name__ == '__main__':
    unittest.main()
