"""
Map post-processing service for SLAM occupancy grids.

Applies morphological operations to clean up occupancy grid maps
for better navigation and visualization.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional
import cv2
import numpy as np
import yaml

log = logging.getLogger(__name__)


class MapProcessor:
    """
    Post-processes SLAM occupancy grid maps using morphological operations.

    Applies opening (erosion→dilation) to remove noise and closing (dilation→erosion)
    to fill gaps, creating cleaner maps for navigation while preserving original data.
    """

    def __init__(
        self,
        opening_kernel_size: int = 3,
        closing_kernel_size: int = 3,
        enabled: bool = True
    ):
        """
        Initialize map processor with configuration.

        Args:
            opening_kernel_size: Kernel size for opening operation (removes noise)
            closing_kernel_size: Kernel size for closing operation (fills gaps)
            enabled: Whether post-processing is enabled
        """
        self.opening_kernel_size = opening_kernel_size
        self.closing_kernel_size = closing_kernel_size
        self.enabled = enabled

    def process_map(
        self,
        input_pgm_path: Path,
        input_yaml_path: Path,
        output_pgm_path: Path,
        output_yaml_path: Path
    ) -> bool:
        """
        Apply post-processing to an occupancy grid map.

        Args:
            input_pgm_path: Path to input .pgm file
            input_yaml_path: Path to input .yaml file
            output_pgm_path: Path to output .pgm file
            output_yaml_path: Path to output .yaml file

        Returns:
            True if processing successful, False otherwise
        """
        if not self.enabled:
            log.info("Map post-processing disabled, skipping")
            return False

        try:
            # Load the occupancy grid image
            if not input_pgm_path.exists():
                log.error(f"Input PGM file not found: {input_pgm_path}")
                return False

            map_image = cv2.imread(str(input_pgm_path), cv2.IMREAD_GRAYSCALE)
            if map_image is None:
                log.error(f"Failed to load PGM image: {input_pgm_path}")
                return False

            log.info(f"Loaded occupancy grid: {map_image.shape}")

            # Apply morphological operations
            processed = self._apply_morphology(map_image)

            # Save processed image
            cv2.imwrite(str(output_pgm_path), processed)
            log.info(f"Saved processed map to: {output_pgm_path}")

            # Copy and update YAML metadata
            if input_yaml_path.exists():
                with open(input_yaml_path, 'r') as f:
                    yaml_data = yaml.safe_load(f)

                # Update image filename in YAML
                yaml_data['image'] = output_pgm_path.name

                with open(output_yaml_path, 'w') as f:
                    yaml.dump(yaml_data, f, default_flow_style=False)

                log.info(f"Saved processed YAML to: {output_yaml_path}")
            else:
                log.warning(f"Input YAML not found: {input_yaml_path}")

            return True

        except Exception as e:
            log.error(f"Error processing map: {e}", exc_info=True)
            return False

    def _apply_morphology(self, image: np.ndarray) -> np.ndarray:
        """
        Apply morphological operations to clean the map.

        Process:
        1. Opening (erosion → dilation): Removes small noise and thin features
        2. Closing (dilation → erosion): Fills small holes and gaps

        Args:
            image: Input grayscale occupancy grid (0=free, 100=occupied, 205=unknown)

        Returns:
            Processed occupancy grid
        """
        # Create a binary mask for occupied cells (darker values in PGM)
        # In PGM: 0=black (occupied), 254=white (free), 205=gray (unknown)
        # We want to process only the occupied/free boundary, not unknown areas

        # Threshold to separate occupied (0-127) from free/unknown (128-255)
        _, binary = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)

        # Apply opening (remove noise - small occupied pixels)
        if self.opening_kernel_size > 0:
            kernel_opening = cv2.getStructuringElement(
                cv2.MORPH_RECT,
                (self.opening_kernel_size, self.opening_kernel_size)
            )
            binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_opening)
            log.debug(f"Applied opening with kernel size {self.opening_kernel_size}")

        # Apply closing (fill gaps - small free space in occupied areas)
        if self.closing_kernel_size > 0:
            kernel_closing = cv2.getStructuringElement(
                cv2.MORPH_RECT,
                (self.closing_kernel_size, self.closing_kernel_size)
            )
            binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_closing)
            log.debug(f"Applied closing with kernel size {self.closing_kernel_size}")

        # Preserve unknown areas from original image
        # Where original was unknown (205), keep it unknown
        unknown_mask = (image == 205)
        result = binary.copy()
        result[unknown_mask] = 205

        return result


# Default processor instance
default_processor = MapProcessor(
    opening_kernel_size=3,
    closing_kernel_size=3,
    enabled=True
)
