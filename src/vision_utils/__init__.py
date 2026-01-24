from .VisionDatas import Settings

if Settings.RoboInfo.Rec == 'AI':
    from .ArisuIntelligence import ArisuIntelligence as Detect_Method
elif Settings.RoboInfo.Rec == 'TV':
    from .TunaVision import TunaVision as Detect_Method
else:
    from .Blob import BlobBased as Detect_Method

from .PreProcess import DATA_DIR

__version__ = '0.2.0'
# ReasonData 0.2.0(20250722A) Updated Preference instead QKjson


__all__ = ['ArisuIntelligence','TunaVision','BlobBased','Detect_Method','DATA_DIR']
