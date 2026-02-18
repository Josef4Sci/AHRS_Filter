"""
AHRS Filters Package
Collection of attitude and heading reference system filters
"""

from .madgwick_ahrs import MadgwickAHRS
from .justa_ahrs import JustaAHRSPure, JustaAHRSv2, JustaAHRSInv, JustaAHRSInvFast, JustaAHRSv3, JustaAHRSv4
from .valenti_ahrs import ValentiAHRS
from .wilson_ahrs import WilsonMadgwickAHRS, AdmirallWilsonAHRS
from .youngsoo_suh_ahrs import YoungSooSuhAHRS
from .jinwu_kf_ahrs import JinWuKFAHRS

__all__ = [
    'MadgwickAHRS',
    'JustaAHRSPure',
    'JustaAHRSv2',
    'JustaAHRSInv',
    'ValentiAHRS',
    'WilsonMadgwickAHRS',
    'AdmirallWilsonAHRS',
    'YoungSooSuhAHRS',
    'JinWuKFAHRS',
    'JustaAHRSInvFast',
    'JustaAHRSv3',
    'JustaAHRSv4',
]
