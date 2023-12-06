"""
The library for applying filters in opencv style, along with some helpers to tune the filter.

Two main parts of the library
* FilterBase and his child classes
* TrackBarHelper class
"""
import dataclasses
import functools
import typing
from typing import Any

import cv2
import numpy as np


@dataclasses.dataclass
class FilterBase:
    """
    The base class for cv filters Defines the common method.

    The class inherent this must also be a data class. This class specifically depends
    on the dataclass field features. There is a pre-defined way to find the MIN MAX values.
    of each field. Look at get_field_range for the name based look up.

    Example:
    -------
    @dataclasses.dataclass
    Class UpperLowerFilter(FilterBase):
        size_upper:int
        size_lower:int
        SIZE_MAX: typing.ClassVar[float] = 255
        SIZE_MIN: typing.ClassVar[float] = 255

    """

    def get_field_range(self, field_name: str) -> tuple[float, float]:
        """
        Get the possible min and max of a field.

        Args
        ----
            field_name (str): string name of one of the class's field

        Raises
        ------
            ValueError: if the name of the field is not in the class

        """
        if field_name not in [f.name for f in dataclasses.fields(self)]:
            raise ValueError(f"Trying to get the range of a field: {field_name}" +
                             " that doesn't exists in data class")

        # Have a strong coupling of the namings of variables
        # TODO in future, require fields to be specific type
        # that capture value, min and max together
        return (
            getattr(self,
                    field_name.split("_")[0].upper() + "_MAX"),
            getattr(self,
                    field_name.split("_")[0].upper() + "_MIN"),
        )

    def update_field_by_name(self, field_name: str, value: float) -> list[str]:
        """
        Update field by its name.

        Args
        ----
            field_name (str): The name of the field
            value (float): New value for the field

        Returns
        -------
            list[str]: List of fields that are linked and should also be called by caller to update

        Raises
        ------
            ValueError: if the name of the field is not in the class

        """
        x_max, x_min = self.get_field_range(field_name)
        value = min(x_max, max(value, x_min))
        setattr(self, field_name, value)
        return []

    def apply_filter(self, bgr_image):
        """
        Apply the cv filter base on field values This should be overwritten by base class.

        The specific implementation depends on each sub class

        Args
        ----
            bgr_image (np.ndarray): The bgr image in cv2's format

        Returns
        -------
            np.Ndarray: Depends on sub class

        """
        return bgr_image

    def ensure_linked_field_smaller(self, field, linked_field, margin=1):
        """
        Make sure the linked field is smaller then given field.

        Relationship is provided by the field name in arguments

        Args
        ----
            field (str): field name
            linked_field (str): linked field name
            margin (int, optional): the margin between to fields. Defaults to 1.

        Returns
        -------
            bool: if linked field's value is updated

        """
        if (getattr(self, field) - getattr(self, linked_field)) < margin:
            # also update counter part
            setattr(self, linked_field, getattr(self, field) - margin)
            return True
        return False

    def ensure_linked_field_larger(self, field, linked_field, margin=1):
        """
        Make sure the linked field is larger then given field.

        Relationship is provided by the field name in arguments

        Args
        ----
            field (str): field name
            linked_field (str): linked field name
            margin (int, optional): the margin between to fields. Defaults to 1.

        Returns
        -------
            bool: if linked field's value is updated

        """
        if (getattr(self, linked_field) - getattr(self, field)) < margin:
            # also update counter part
            setattr(self, linked_field, getattr(self, field) + margin)
            return True
        return False


@dataclasses.dataclass
class HSVFilter_SVBase(FilterBase):
    """S and V channel in HSV filter."""

    s_high: float = 255
    s_low: float = 118
    v_high: float = 255
    v_low: float = 170

    S_MAX: typing.ClassVar[float] = 255
    S_MIN: typing.ClassVar[float] = 0
    V_MAX: typing.ClassVar[float] = 255
    V_MIN: typing.ClassVar[float] = 0

    def update_field_by_name(self, field_name: str, value: float) -> list[str]:
        """
        Update value in field by its name.

        The counter part of the field will also be updated

        Args
        ----
            field_name (str): field name
            value (float): new values

        Returns
        -------
            list[str]: counter part field that also have been update

        """
        super().update_field_by_name(field_name, value)

        # The custom linked attribute updating
        channel, side = field_name.split("_")
        if side == "high":
            counter_part = channel + "_low"
            if getattr(self, counter_part) > getattr(self, field_name):
                # also update counter part
                setattr(self, counter_part, value)
        else:
            counter_part = channel + "_high"
            if getattr(self, counter_part) < getattr(self, field_name):
                # also update counter part
                setattr(self, counter_part, value)
        return [counter_part]

    def apply_filter(self, bgr_image):
        """
        Apply the HSV filter (only limiting in S and V channel).

        Args
        ----
            bgr_image (np.ndarray): input image

        Returns
        -------
            np.ndarray: The HSV mask

        """
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        return cv2.inRange(hsv_image, (0, self.s_low, self.v_low), (179, self.s_high, self.v_high))


@dataclasses.dataclass
class HOnlyFilter(FilterBase):
    """This class only does the H in HSV filter."""

    h_high: float = 26
    h_low: float = 0

    # The exact names of these const are referenced later.
    H_MAX: typing.ClassVar[float] = 179
    H_MIN: typing.ClassVar[float] = 0

    # name: typing.ClassVar[str] = ""
    def update_field_by_name(self, field_name: str, value: float) -> list[str]:
        """Instance specific implementation."""
        super().update_field_by_name(field_name, value)

        # The custom linked attribute updating
        channel, side = field_name.split("_")
        if side == "high":
            counter_part = channel + "_low"
            if getattr(self, counter_part) > getattr(self, field_name):
                # also update counter part
                setattr(self, counter_part, value)
        else:
            counter_part = channel + "_high"
            if getattr(self, counter_part) < getattr(self, field_name):
                # also update counter part
                setattr(self, counter_part, value)
        return [counter_part]

    def apply_filter(self, bgr_image):
        """
        Apply only the H channel for hsv filter.

        Args
        ----
            bgr_image (np.ndarray): numpy array

        Returns
        -------
            np.ndarray: mask after applying the filter

        """
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        return cv2.inRange(hsv_image, (self.h_low, 0, 0), (self.h_high, 255, 255))


@dataclasses.dataclass
class RedHFilter(FilterBase):
    """
    Specific H only filter for red color (H_high and H_low wrapped around).

    Since RED is at two ends of the HSV filter, actually need the filter value to wrap-around.
    """

    h_low: float = 170
    h_high: float = 20

    H_MAX: typing.ClassVar[float] = 179
    H_MIN: typing.ClassVar[float] = 0

    def apply_filter(self, bgr_image):
        """
        Sspecial way of applying HSV filter with lower half for RED, then upper half.

        Args
        ----
            bgr_image input image

        Returns
        -------
            combined mask (or combined)

        """
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        lower_filter = cv2.inRange(hsv_image, (self.h_low, 0, 0), (self.H_MAX, 255, 255))

        upper_filter = cv2.inRange(hsv_image, (self.H_MIN, 0, 0), (self.h_high, 255, 255))

        return cv2.bitwise_or(lower_filter, upper_filter)


@dataclasses.dataclass
class HSVFilter(FilterBase):
    """Full hsv filter with all three channels."""

    h_high: float = 26
    h_low: float = 0
    s_high: float = 255
    s_low: float = 118
    v_high: float = 255
    v_low: float = 170

    # The exact names of these const are referenced later.
    H_MAX: typing.ClassVar[float] = 179
    H_MIN: typing.ClassVar[float] = 0
    S_MAX: typing.ClassVar[float] = 255
    S_MIN: typing.ClassVar[float] = 0
    V_MAX: typing.ClassVar[float] = 255
    V_MIN: typing.ClassVar[float] = 0

    def update_field_by_name(self, field_name: str, value: float) -> list[str]:
        """Update a filed (and update its linked counter part if needed)."""
        super().update_field_by_name(field_name, value)

        # The custom linked attribute updating
        channel, side = field_name.split("_")
        if side == "high":
            counter_part = channel + "_low"
            if getattr(self, counter_part) > getattr(self, field_name):
                # also update counter part
                setattr(self, counter_part, value - 1)
        else:
            counter_part = channel + "_high"
            if getattr(self, counter_part) < getattr(self, field_name):
                # also update counter part
                setattr(self, counter_part, value + 1)
        return [counter_part]

    def apply_filter(self, bgr_image):
        """
        Apply the HSV filter.

        Returns
        -------
            HSV mask after applying

        """
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        return cv2.inRange(
            hsv_image,
            (self.h_low, self.s_low, self.v_low),
            (self.h_high, self.s_high, self.v_high),
        )


@dataclasses.dataclass
class BlobDetector(FilterBase):
    """
    Filter for blob detection.

    Only parameter of threshold and area are exposed
    """

    maxThreshold: int = 101
    minThreshold: int = 11

    maxArea: int = 9999
    minArea: int = 601

    MAXTHRESHOLD_MAX: typing.ClassVar[float] = 250
    MAXTHRESHOLD_MIN: typing.ClassVar[float] = 30

    MINTHRESHOLD_MAX: typing.ClassVar[float] = 230
    MINTHRESHOLD_MIN: typing.ClassVar[float] = 2

    MAXAREA_MAX: typing.ClassVar[float] = 10000
    MAXAREA_MIN: typing.ClassVar[float] = 500

    MINAREA_MAX: typing.ClassVar[float] = 3000
    MINAREA_MIN: typing.ClassVar[float] = 50

    # Overriding init for dataclass is bad
    def __post_init__(self):
        """Specific post init to be called by dataclass to make the parameter object needed."""
        super().__init__()

        self.params = cv2.SimpleBlobDetector_Params()

        self.detector = cv2.SimpleBlobDetector_create(self.params)

    def update_field_by_name(self, field_name: str, value: float) -> list[str]:
        """Similar to parents's version with special linkages between fields."""
        super().update_field_by_name(field_name, value)

        # Additionally update the param object's value
        setattr(self.params, field_name, getattr(self, field_name))
        self.detector = cv2.SimpleBlobDetector_create(self.params)

        if field_name == "maxThreshold":
            if self.ensure_linked_field_smaller(field_name, "minThreshold", 2):
                return ["minThreshold"]

        if field_name == "minThreshold":
            if self.ensure_linked_field_larger(field_name, "maxThreshold", 2):
                return ["maxThreshold"]

        if field_name == "maxArea":
            if self.ensure_linked_field_smaller(field_name, "minArea", 10):
                return ["minArea"]

        if field_name == "minArea":
            if self.ensure_linked_field_larger(field_name, "maxArea", 10):
                return ["maxArea"]
        return []

    def apply_filter(self, bgr_image) -> tuple[Any, list[cv2.KeyPoint]]:
        """
        Specific one for appling blob detector.

        Args
        ----
            bgr_image (np.ndarray): input image

        Returns
        -------
            tuple[Any, list[cv2.KeyPoint]]: Tuple of labeled image
            (with circle and center dot) and a list of keypoints.

        """
        keypoints = self.detector.detect(bgr_image)

        for keypoint in keypoints:
            x, y = keypoint.pt
            x = int(x)
            y = int(y)

            cv2.circle(bgr_image, (x, y), 5, (100, 100, 255), -1)

        im_with_keypoints = cv2.drawKeypoints(
            bgr_image,
            keypoints,
            np.array([]),
            (0, 0, 255),
            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )

        return im_with_keypoints, keypoints


class TrackBarHelper:

    def scale_image_down(self, image: np.ndarray, expected_width):
        """
        Scale the image down so it's display-able on smaller screen when having track bars.

        Args
        ----
            image (np.ndarray): cv2 image
            expected_width (int): width after scaling

        Returns
        -------
            np.ndarray: scaled image

        """
        # We only care about width, doesn't care about height or color channel numbers
        _, width, _ = image.shape

        scale_down_factor = expected_width / width
        return cv2.resize(image, (0, 0), fx=scale_down_factor, fy=scale_down_factor)

    # Node: the order of arguments matter.
    def update_field_callback(self, window_name: str, filter: FilterBase, field_name: str,
                              new_value):
        """
        Ccallback function to update a field when its value changes.

        Should use argument binding before give it to cv2.createTrackbar.
        Last argument is the one given by cv2 during callback.

        Args:
        ----
            window_name (str): name of the
            filter (FilterBase): The filer
            field_name (str): name of the field
            new_value (float): value

        """
        # Problem with cv2 setTrackbarPos is it also trigger callback (instead of
        # just user hitting it)
        # So this would have cause a infinite loop
        # In practice, the callback is only called when value is changed.
        # So what will happen is a callback to change h_high will not re-trigger itself
        # internally. However this this h_high callback also change h_low, it will
        # trigger a h_low callback. Luckly, since value is set, setting h_low won't trigger
        # another h_high callback
        # In theory this is actually very very bad. But in practice it worked.

        linked_attrs = filter.update_field_by_name(field_name, new_value)
        actual_set_value = getattr(filter, field_name)
        print(f"Setting {field_name} to {actual_set_value}")
        cv2.setTrackbarPos(field_name, window_name, actual_set_value)

        for linked_attr in linked_attrs:
            print(f"Also update linked attr {linked_attr}")
            linked_value = getattr(filter, linked_attr)
            cv2.setTrackbarPos(linked_attr, window_name, linked_value)

    def setup_cv_trackbar(self, filter: FilterBase, window_name: str = "trackbars"):
        """
        Set up trackbars on window for each field in the Filter object.

        Args:
        ----
            filter (FilterBase): The filter object
            window_name (str, optional): Name of the window to attach the trackbar
            to Will create the window internally.
            Defaults to "trackbars".

        """
        default_value_map = {}
        cv2.namedWindow(window_name)

        for field in dataclasses.fields(filter):
            name = field.name
            print(f"\n-------\n  Adding field {name}")
            field_max, field_min = filter.get_field_range(name)
            # Shouldn't use lambda here, as lambda creates links not copies
            # binded_callback = lambda value: self.update_field_callback(name,value)

            # Note this binding is in orders, from left to right.
            binded_callback = functools.partial(self.update_field_callback, window_name, filter,
                                                name)

            print(f"max min {field_max} , {field_min}\n{binded_callback}")

            # When creating trackbar with min bigger then 0, or max smaller then 0.
            # the creating process will call setTrackbar to snap it to limit, but
            # will trigger a callback. Thus
            # The default value needs to to be remembered.
            default_value_map[name] = getattr(filter, name)
            # print(f"callback {binded_callback}")
            cv2.createTrackbar(name, window_name, field_min, field_max, binded_callback)

        for field in dataclasses.fields(filter):
            # Must do this after all bars being done. Or the set trackbar could trigger
            # a callback which cause the linked_attr's trackbar (not existing) to be sets
            print(f"-- Initializeing {field.name} to {default_value_map[field.name]} ")
            cv2.setTrackbarPos(field.name, window_name, default_value_map[field.name])
