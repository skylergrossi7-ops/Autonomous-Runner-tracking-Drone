"""OpenCV camera input used by the perception pipeline."""

from __future__ import annotations

from typing import Union

import cv2


class CameraSource:
    """Open a camera or video source and return resized BGR frames.

    The default source, ``0``, is the laptop's first camera. A string path can
    be supplied later to test with a recorded video.
    """

    def __init__(
        self,
        source: Union[int, str] = 0,
        width: int = 960,
        height: int = 540,
        backend: int = cv2.CAP_ANY,
    ) -> None:
        if width <= 0 or height <= 0:
            raise ValueError("Camera width and height must be positive")

        self.source = source
        self.width = width
        self.height = height
        self.backend = backend
        self._capture = None

    @property
    def is_open(self) -> bool:
        """Return whether the underlying OpenCV source is open."""
        return self._capture is not None and self._capture.isOpened()

    def open(self) -> None:
        """Open the configured source.

        Calling this method when the source is already open has no effect.
        """
        if self.is_open:
            return

        self._capture = cv2.VideoCapture(self.source, self.backend)
        if not self._capture.isOpened():
            self._capture.release()
            self._capture = None
            raise RuntimeError(
                f"Could not open camera/video source {self.source!r}. "
                "Check that the device exists and is not being used by "
                "another application."
            )

        # These settings are requests. Some cameras select the closest
        # supported resolution instead, so frames are still resized in read().
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def read(self):
        """Read and resize one BGR frame.

        Raises:
            RuntimeError: If the source is closed or a frame cannot be read.
        """
        if not self.is_open:
            raise RuntimeError("Camera source is not open. Call open() first.")

        received, frame = self._capture.read()
        if not received or frame is None:
            raise RuntimeError(
                f"Could not read a frame from source {self.source!r}"
            )

        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        return frame

    def release(self) -> None:
        """Release the camera safely."""
        if self._capture is not None:
            self._capture.release()
            self._capture = None

    def __enter__(self) -> "CameraSource":
        self.open()
        return self

    def __exit__(self, exception_type, exception, traceback) -> None:
        self.release()
