from pathlib import Path
import shlex,subprocess,os,shutil
r=Path(__file__).resolve().parents[1];b=r/'build/go2w_rl_bridge';d=r/'build/go2w_rl_bridge_inline';d.mkdir(exist_ok=True)
flags={}
for line in (b/'CMakeFiles/go2w_rl_bridge.dir/flags.make').read_text().splitlines():
 if ' = ' in line:
  k,v=line.split(' = ',1);flags[k]=shlex.split(v.replace('/home/unitree/matrix_go2w_lcm_demo',str(r)))
obj=d/'main.cpp.o';binary=d/'go2w_rl_bridge'
subprocess.run(['/usr/bin/c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-c',str(r/'controllers/go2w_rl_bridge/src/main.cpp'),'-o',str(obj)],check=True)
a=shlex.split((b/'CMakeFiles/go2w_rl_bridge.dir/link.txt').read_text());a[a.index('CMakeFiles/go2w_rl_bridge.dir/src/main.cpp.o')]=str(obj);a[a.index('-o')+1]=str(binary);subprocess.run(a,check=True)
shutil.copy2(binary,r/'install/bin/go2w_rl_bridge.next');os.replace(r/'install/bin/go2w_rl_bridge.next',r/'install/bin/go2w_rl_bridge')
