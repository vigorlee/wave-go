"""Compile exact native constructor/reset/empty-scan guards against poisoned state."""
import json, re, subprocess, tempfile, unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
FIX=ROOT/'artifacts/rviz_alignment_fix_20261004_210343'
def body(text, marker):
 start=text.index('{',text.index(marker));depth=0
 for i in range(start,len(text)):
  if text[i]=='{':depth+=1
  elif text[i]=='}':
   depth-=1
   if depth==0:return text[start:i+1]
 raise ValueError(marker)
def cpp_for(base):
 imu=(base/'genisom_roamerx_open/src/slam/src/src/process/imu_process.cpp').read_text()
 mapping=(base/'genisom_roamerx_open/src/slam/src/src/mapping_alg.cpp').read_text()
 header=(base/'genisom_roamerx_open/src/slam/src/include/mapping_alg.h').read_text()
 ctor_start=imu.index('ImuProcess::ImuProcess()');ctor_end=imu.index('ImuProcess::~ImuProcess()');ctor=imu[ctor_start:ctor_end]
 reset1='void ImuProcess::Reset()'+body(imu,'void ImuProcess::Reset()')
 reset2='void ImuProcess::reset()'+body(imu,'void ImuProcess::reset()')
 process=body(imu,'void ImuProcess::Process(');empty=body(process,'if (meas.imu.empty())')
 declaration=re.search(r'bool\s+lidar_pushed[^;]*;',header).group(0)
 reset_body=body(mapping,'void MappingAlg::reset()');flag_reset=re.search(r'lidar_pushed\s*=\s*false;',reset_body)
 flag_reset=flag_reset.group(0) if flag_reset else ''
 run=body(mapping,'void MappingAlg::run()');guard=body(run,'if (Measures.imu.empty())') if 'if (Measures.imu.empty())' in run else ''
 return r'''
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <memory>
#include <new>
#include <vector>
namespace robot::slam {
 using Vec3d=Eigen::Vector3d;const Vec3d Zero3d=Vec3d::Zero();const Eigen::Matrix3d Eye3d=Eigen::Matrix3d::Identity();
 struct ImuMessage {double timestamp=0;};struct PointCloudType {std::vector<int> points;void clear(){points.clear();}size_t size()const{return points.size();}};
}
Eigen::Matrix<double,12,12> process_noise_cov(){return Eigen::Matrix<double,12,12>::Identity();}
struct ImuProcess {
 bool b_first_frame_,imu_need_init_;double start_timestamp_;int init_iter_num;
 Eigen::Matrix<double,12,12> Q;Eigen::Vector3d cov_acc,cov_gyr,cov_bias_gyr,cov_bias_acc,mean_acc,mean_gyr,angvel_last,acc_s_last,Lidar_T_wrt_IMU;
 Eigen::Matrix3d Lidar_R_wrt_IMU;double last_lidar_end_time_;
 std::shared_ptr<robot::slam::ImuMessage> last_imu_;std::shared_ptr<robot::slam::PointCloudType> cur_pcl_un_;
 std::deque<int> v_imu_;std::vector<int> IMUpose;
 ImuProcess();void Reset();void reset();
};
'''+ctor+reset1+reset2+r'''
struct Measure {std::vector<int> imu;double lidar_end_time=10;};
void process_empty(const Measure& meas,std::shared_ptr<robot::slam::PointCloudType> cur_pcl_un_){if(meas.imu.empty())
'''+empty+r'''
}
#define RCLCPP_WARN(...) do {} while(false)
struct MappingShim {
'''+declaration+'\nvoid reset(){'+flag_reset+'}\n'+r'''
 int updates=0;Measure Measures;std::shared_ptr<robot::slam::PointCloudType> feats_undistort=std::make_shared<robot::slam::PointCloudType>();
 void scan(){
'''+('if(Measures.imu.empty())'+guard if guard else '')+r'''
  ++updates;
 }
};
int main(){
 alignas(ImuProcess) unsigned char memory[sizeof(ImuProcess)];std::memset(memory,0xA5,sizeof(memory));auto* imu=new(memory) ImuProcess();
 const double zero=0;bool ctor_last=std::memcmp(&imu->last_lidar_end_time_,&zero,sizeof(double))==0;bool ctor_acc=std::memcmp(imu->acc_s_last.data(),Eigen::Vector3d::Zero().eval().data(),3*sizeof(double))==0;
 imu->last_lidar_end_time_=77;imu->acc_s_last.setConstant(8);imu->Reset();bool reset1=imu->last_lidar_end_time_==0&&imu->acc_s_last.isZero();
 imu->last_lidar_end_time_=78;imu->acc_s_last.setConstant(9);imu->reset();bool reset2=imu->last_lidar_end_time_==0&&imu->acc_s_last.isZero();
 imu->~ImuProcess();
 alignas(MappingShim) unsigned char mm[sizeof(MappingShim)];std::memset(mm,0xA5,sizeof(mm));auto* map=new(mm) MappingShim;
 unsigned char flag;std::memcpy(&flag,&map->lidar_pushed,1);map->lidar_pushed=true;map->reset();bool flag_reset=!map->lidar_pushed;
 auto cloud=std::make_shared<robot::slam::PointCloudType>();cloud->points.assign(7,1);process_empty(Measure{},cloud);map->feats_undistort->points.assign(7,1);map->scan();
 std::cout<<"{\"ctor_last_initialized\":"<<ctor_last<<",\"ctor_acc_initialized\":"<<ctor_acc<<",\"Reset_initializes\":"<<reset1<<",\"reset_initializes\":"<<reset2<<",\"lidar_flag_storage_byte\":"<<(int)flag<<",\"lidar_reset_false\":"<<flag_reset<<",\"empty_process_old_cloud_remaining\":"<<cloud->size()<<",\"empty_run_updates\":"<<map->updates<<",\"empty_run_old_cloud_remaining\":"<<map->feats_undistort->size()<<"}\n";
 map->~MappingShim();return 0;
}
'''
class StateHygiene(unittest.TestCase):
 def test_native_uninitialized_and_empty_scan_paths(self):
  reports={}
  with tempfile.TemporaryDirectory(prefix='lio_hygiene_proof_') as folder:
   tmp=Path(folder)
   for label,base in [('legacy',FIX/'source_before_state_hygiene'),('fixed',ROOT)]:
    cpp=tmp/(label+'.cpp');exe=tmp/label;cpp.write_text(cpp_for(base))
    subprocess.run(['/usr/bin/g++','-O1','-std=c++17','-I/usr/include/eigen3',str(cpp),'-o',str(exe)],check=True,capture_output=True,text=True)
    reports[label]=json.loads(subprocess.run([str(exe)],check=True,capture_output=True,text=True).stdout)
  old,new=reports['legacy'],reports['fixed']
  self.assertEqual(old['lidar_flag_storage_byte'],0xA5)
  for k in ('ctor_last_initialized','ctor_acc_initialized','Reset_initializes','reset_initializes','lidar_reset_false'):self.assertFalse(old[k],k);self.assertTrue(new[k],k)
  self.assertEqual(new['lidar_flag_storage_byte'],0)
  self.assertEqual(old['empty_process_old_cloud_remaining'],7);self.assertEqual(new['empty_process_old_cloud_remaining'],0)
  self.assertEqual(old['empty_run_updates'],1);self.assertEqual(new['empty_run_updates'],0);self.assertEqual(new['empty_run_old_cloud_remaining'],0)
  report=dict(passed=True,exact_native_bodies_extracted=True,poisoned_storage_byte=0xA5,indeterminate_values_not_read_as_typed_before_fix=True,results=reports,claim='Deterministic initialization and stale-cloud rejection proven; whole-route vertical drift cause not established')
  (FIX/'native_lio_bug_audit_20261004/state_hygiene_validation.json').write_text(json.dumps(report,indent=2)+'\n')
if __name__=='__main__':unittest.main()
