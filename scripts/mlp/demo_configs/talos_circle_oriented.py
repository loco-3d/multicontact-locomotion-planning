TIMEOPT_CONFIG_FILE = "cfg_softConstraints_talos.yaml"
from .common_talos import *
SCRIPT_PATH = "memmo"
ENV_NAME = "multicontact/ground"

EFF_T_PREDEF = 0.2
EFF_T_DELAY = 0.05
p_max = 0.07

YAW_ROT_GAIN = 0.01
end_effector_method = "limbRRToptimized" 