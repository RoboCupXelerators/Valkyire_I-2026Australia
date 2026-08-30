import pytest

import vision_utils
import vision_utils.PreProcess as preprocess
import vision_utils.Vision as vision_module
import vision_utils.XeleratorVision as xelerator_module


def test_active_vision_modules_share_merged_settings_singleton():
    assert preprocess.Settings is vision_utils.Settings
    assert vision_module.Settings is vision_utils.Settings
    assert xelerator_module.Settings is vision_utils.Settings


def test_backend_import_error_is_not_silently_downgraded():
    def broken_loader(*_args, **_kwargs):
        raise vision_utils.VisionBackendImportError("broken backend")

    with pytest.raises(vision_utils.VisionBackendImportError, match="broken backend"):
        vision_utils._select_backend("XV", loader=broken_loader)


def test_missing_hailo_is_visible_when_blob_fallback_is_used():
    class BlobFallback:
        __name__ = "BlobFallback"

    def missing_hailo(*_args, **_kwargs):
        raise vision_utils._OptionalBackendDependencyMissing("AI requires optional module 'hailo_platform'")

    backend, status = vision_utils._select_backend(
        "AI",
        loader=missing_hailo,
        blob_loader=lambda: BlobFallback,
    )

    assert backend is BlobFallback
    assert status == {
        "requested": "AI",
        "active": "BlobFallback",
        "degraded": True,
        "reason": "AI requires optional module 'hailo_platform'",
    }


def test_unknown_backend_is_rejected_instead_of_falling_back():
    with pytest.raises(ValueError, match="Unsupported RoboInfo.Rec"):
        vision_utils._select_backend("typo")
