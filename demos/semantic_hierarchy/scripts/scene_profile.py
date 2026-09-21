"""Separate read-only code resources from independent recorded-map datasets."""
import json, os
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
DATA_ROOT=Path(os.environ.get('SCENE_DATASET_ROOT',str(ROOT))).resolve()
PROFILE=json.loads((DATA_ROOT/'profile.json').read_text()) if (DATA_ROOT/'profile.json').exists() else {}
CHINESE=os.environ.get('SCENE_CHINESE','0')=='1'
