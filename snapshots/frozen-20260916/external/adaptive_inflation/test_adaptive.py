import numpy as np
import pytest
from adaptive_core import *

def test_width_changes_only_comfort_not_body():
    narrow=radius_policy(1.15,.2);wide=radius_policy(3.,.2)
    assert narrow['radius']<wide['radius']
    assert narrow['radius']>=narrow['hard_radius']>=.35+.08
    assert radius_policy(.7,.2)['too_narrow']

def test_speed_braking_is_not_erased_by_narrow_passage():
    slow=radius_policy(1.3,0);fast=radius_policy(1.3,.8)
    assert fast['radius']>slow['radius'] and fast['too_narrow']
    assert fast['hard_radius']==pytest.approx(.35+.08+.8*.25+.8**2/2)

def test_missing_side_is_not_a_measured_corridor():
    x=np.linspace(0,3,121);p=np.column_stack([x,np.full_like(x,1),np.ones_like(x)])
    g=build_grid(p,np.zeros(len(p),np.uint8),0)
    width,_=corridor_width(g,[0,0],0)
    assert width is None
    assert not radius_policy(width,0)['width_valid']

def test_bilateral_raw_corridor_and_inflation():
    x=np.linspace(-1,3,161);p=np.concatenate([np.column_stack([x,np.full_like(x,y),np.ones_like(x)]) for y in [-.75,.75]])
    g=build_grid(p,np.zeros(len(p),np.uint8),0);w,_=corridor_width(g,[0,0],0)
    assert 1.3<=w<=1.6
    result,grids=analyze(g,[0,0]);assert np.all(grids['inflated'][g['occupancy']])
    assert result['hard_radius']<=result['radius']

def test_negative_z_not_discarded_and_unknown_not_free():
    p=np.array([[0,0,-.8],[1,1,-1.1],[0,1,-1.15]],np.float32)
    g=build_grid(p,np.array([5,6,6],np.uint8),-1.2)
    assert g['occupancy'].sum()==1 and not g['observed'].all()

def test_people_margin_larger_than_static():
    p=np.array([[0,0,1.],[3,0,1.]],np.float32)
    g=build_grid(p,np.array([4,5],np.uint8),0);res,grids=analyze(g,[1.5,1.5],speed=0)
    # Compare same-distance cells on the open upper side of each obstacle.
    def cell(x,y):
        ij=np.floor((np.array([x,y])-g['origin'])/.05).astype(int);return tuple(ij[::-1])
    assert grids['inflated'][cell(0,1.)] and not grids['inflated'][cell(3,1.)]

def test_ascii_pcd_and_truncation(tmp_path):
    h='VERSION .7\nFIELDS x y z label\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA '
    p=tmp_path/'a.pcd';p.write_text(h+'ascii\n0 1 -2 4\n1 2 -3 5\n')
    points,l,_=read_pcd(p);assert l.tolist()==[4,5] and points[0,2]==-2
    p.write_bytes((h+'binary\n').encode()+b'123')
    with pytest.raises(ValueError):read_pcd(p)
