"""Verify density adaptation, exact point coverage, and retained wide gaps."""
import importlib.util
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('adaptive',root/'scripts/build_adaptive_voxels.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
a,b=np.meshgrid(np.arange(0,1,.04),np.arange(0,1,.04));dense=np.c_[a.ravel(),b.ravel(),np.zeros(a.size)]
a,b=np.meshgrid(np.arange(3,4,.17),np.arange(0,1,.17));sparse=np.c_[a.ravel(),b.ravel(),np.zeros(a.size)]
xyz=np.vstack([dense,sparse]);c,s,n,_,_=m.build(xyz);m.verify(xyz,c,s,n)
assert np.median(s[c[:,0]<2])<np.median(s[c[:,0]>2])
assert not np.any((c[:,0]>1.3)&(c[:,0]<2.7))
print('PASS: dense/fine, sparse/coarse, exact unique coverage, wide gap retained')
