"""Allow any test module to run independently from this bundle."""
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
