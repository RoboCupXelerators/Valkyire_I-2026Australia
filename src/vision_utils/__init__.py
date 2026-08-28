"""vision_utils package entry and explicit backend registry."""

from .merged_settings import Settings

# Logger ownership remains in ReasonData; legacy VisionDatas is intentionally
# not imported here because loading it must not change configuration state.
try:
    from ReasonData import logger as logger  # type: ignore
except Exception:  # pragma: no cover
    import logging

    logger = logging.getLogger(__name__)
    if not logger.handlers:
        logger.addHandler(logging.NullHandler())


class VisionBackendImportError(RuntimeError):
    """The configured vision backend could not be imported safely."""


class _OptionalBackendDependencyMissing(VisionBackendImportError):
    """A configured accelerator is absent and may use the Blob fallback."""


_BACKENDS = {
    "AI": ("ArisuIntelligence", "ArisuIntelligence"),
    "TI": ("TunaIntelligence", "TunaIntelligence"),
    "HV": ("HybridVision", "HybridVision"),
    "HYBRID": ("HybridVision", "HybridVision"),
    "XV": ("XeleratorVision", "XeleratorVision"),
    "SY": ("LunaIntelligence", "SimYoloVision"),
    "SIMYOLO": ("LunaIntelligence", "SimYoloVision"),
}
_OPTIONAL_ACCELERATOR_MODULES = {"AI": frozenset({"hailo_platform"})}
_BLOB_REQUESTS = {"", "BB", "BLOB", "BLOBBASED"}


def _safe_import(path: str, name: str, *, backend: str):
    """Load a backend without hiding source or configuration errors."""
    try:
        mod = __import__(path, fromlist=[name])
        return getattr(mod, name)
    except ModuleNotFoundError as exc:
        optional_modules = _OPTIONAL_ACCELERATOR_MODULES.get(backend, frozenset())
        if exc.name in optional_modules:
            raise _OptionalBackendDependencyMissing(
                f"{backend} requires optional module {exc.name!r}"
            ) from exc
        raise VisionBackendImportError(
            f"configured backend {backend} could not import {path}.{name}: {exc}"
        ) from exc
    except Exception as exc:
        raise VisionBackendImportError(
            f"configured backend {backend} could not import {path}.{name}: {exc}"
        ) from exc


def _load_blob_backend():
    try:
        from .Blob import BlobBased
    except Exception as exc:
        raise VisionBackendImportError(f"Blob fallback is unavailable: {exc}") from exc
    return BlobBased


def _select_backend(requested: str, *, loader=None, blob_loader=None):
    """Return the selected backend and a status object suitable for diagnostics."""

    requested = str(requested or "").upper()
    loader = loader or _safe_import
    blob_loader = blob_loader or _load_blob_backend

    if requested in _BLOB_REQUESTS:
        backend = blob_loader()
        return backend, {
            "requested": requested or "BB",
            "active": backend.__name__,
            "degraded": False,
            "reason": None,
        }

    spec = _BACKENDS.get(requested)
    if spec is None:
        raise ValueError(
            f"Unsupported RoboInfo.Rec={requested!r}; use AI, TI, HV, XV, SY, or BB."
        )

    module_name, class_name = spec
    try:
        backend = loader(f"{_PKG}.{module_name}", class_name, backend=requested)
    except _OptionalBackendDependencyMissing as exc:
        backend = blob_loader()
        reason = str(exc)
        logger.warning(f"vision_utils: {reason}; using BlobBased fallback")
        return backend, {
            "requested": requested,
            "active": backend.__name__,
            "degraded": True,
            "reason": reason,
        }

    return backend, {
        "requested": requested,
        "active": backend.__name__,
        "degraded": False,
        "reason": None,
    }


# Prefer package name over module name so direct imports like
# `import src.vision_utils.__init__` still resolve sibling modules correctly.
_PKG = __package__ or __name__.rpartition('.')[0] or __name__
_REC = str(getattr(Settings.RoboInfo, "Rec", "")).upper()
Detect_Method, BACKEND_STATUS = _select_backend(_REC)


def get_backend_status() -> dict:
    """Return a copy so callers can expose backend degradation safely."""

    return dict(BACKEND_STATUS)

# PreProcess depends on cv2; make it optional so SIM/dev machines can import vision_utils.
try:
    from .PreProcess import DATA_DIR
except Exception as e:  # pragma: no cover
    logger.warning(f"vision_utils: PreProcess unavailable (likely missing cv2): {e}")
    DATA_DIR = None

__version__ = '0.2.0'
# ReasonData 0.2.0(20250722A) Updated Preference instead QKjson


__all__ = [
    'Settings', 'Detect_Method', 'BACKEND_STATUS', 'get_backend_status', 'DATA_DIR',
    'VisionBackendImportError',
]
