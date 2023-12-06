import pytest

from teleop_sensing import cv_filters
import dataclasses
import cv2


def test_filter_base():
    filter_base_only = cv_filters.FilterBase()

    # This should throw exception
    with pytest.raises(ValueError):
        filter_base_only.get_field_range("anything")
    with pytest.raises(ValueError):
        filter_base_only.update_field_by_name("anything", 0)


def test_hsv_filter():
    hsv_filter = cv_filters.HSVFilter(20, 10, 20, 10, 20, 10)

    hsv_filter.update_field_by_name("h_high", 30)
    assert hsv_filter.h_high == 30
    linked_filed = hsv_filter.update_field_by_name("h_high", 5)
    assert linked_filed == ["h_low"]
    assert hsv_filter.h_high > hsv_filter.h_low
    linked_filed = hsv_filter.update_field_by_name("v_low", 100)
    assert linked_filed == ["v_high"]
    assert hsv_filter.v_high > hsv_filter.v_low


def test_setup_trackbar():
    test_cv2_window = "test_window"
    h_filter = cv_filters.HOnlyFilter(100, 10)

    helper = cv_filters.TrackBarHelper()
    helper.setup_cv_trackbar(h_filter, window_name="test_window")

    for field in dataclasses.fields(h_filter):
        name = field.name
        assert cv2.getTrackbarPos(name, test_cv2_window) == getattr(h_filter, name)
