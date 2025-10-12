"""
Map quality analysis service for SLAM monitoring.

Provides comprehensive quality metrics for SLAM mapping including:
- Phase 1: Scan processing quality (point count, density, coverage)
- Phase 2: Map structure quality (to be implemented)
- Phase 3: Localization quality (to be implemented)
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Dict, Any, List, Optional
from enum import Enum
import numpy as np

log = logging.getLogger(__name__)


class QualityLevel(Enum):
    """Quality level indicators."""
    EXCELLENT = "excellent"
    GOOD = "good"
    FAIR = "fair"
    POOR = "poor"
    CRITICAL = "critical"


@dataclass
class ScanQualityMetrics:
    """Metrics for scan processing quality (Phase 1)."""

    # Point count metrics
    point_count: int = 0
    min_point_count: int = 100  # Minimum acceptable points per scan
    expected_point_count: int = 360  # Expected points for typical lidar

    # Scan density metrics
    scan_density: float = 0.0  # Points per degree
    min_scan_density: float = 0.5  # Minimum acceptable density
    expected_scan_density: float = 1.0  # Expected density for good scans

    # Angular coverage metrics
    angular_coverage: float = 0.0  # Percentage of 360 degrees covered
    min_angular_coverage: float = 0.7  # Minimum 70% coverage required
    coverage_gaps: List[tuple] = field(default_factory=list)  # List of (start_angle, end_angle) gaps
    max_gap_size: float = 0.0  # Largest gap in degrees

    # Range quality metrics
    mean_range: float = 0.0
    median_range: float = 0.0
    range_std: float = 0.0
    valid_range_ratio: float = 0.0  # Ratio of points within valid range

    # Temporal metrics
    timestamp: float = 0.0
    scan_rate: float = 0.0  # Scans per second
    expected_scan_rate: float = 10.0  # Expected rate (Hz)

    # Quality assessment
    quality_level: QualityLevel = QualityLevel.GOOD
    quality_score: float = 1.0  # 0.0 to 1.0
    issues: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)


@dataclass
class MapStructureMetrics:
    """Metrics for map structure quality (Phase 2)."""

    # Map size and coverage
    map_width: int = 0
    map_height: int = 0
    resolution: float = 0.05  # meters per pixel
    total_cells: int = 0

    # Occupancy distribution
    occupied_cells: int = 0
    free_cells: int = 0
    unknown_cells: int = 0
    occupied_ratio: float = 0.0  # Ratio of occupied to known cells
    free_ratio: float = 0.0
    unknown_ratio: float = 0.0
    explored_ratio: float = 0.0  # Ratio of explored (free + occupied) to total

    # Map entropy and noise
    entropy: float = 0.0  # Shannon entropy of occupancy distribution
    noise_score: float = 0.0  # Measure of isolated occupied cells (noise)

    # Wall quality metrics
    wall_sharpness: float = 0.0  # Average gradient magnitude at walls (higher = sharper)
    wall_thickness_avg: float = 0.0  # Average wall thickness in pixels
    wall_thickness_std: float = 0.0  # Std dev of wall thickness

    # Feature density
    corner_count: int = 0  # Number of detected corners
    edge_count: int = 0  # Number of detected edges
    feature_density: float = 0.0  # Features per square meter

    # Map quality indicators
    has_walls: bool = False  # Whether map contains any walls
    is_empty: bool = True  # Whether map is completely unexplored

    # Quality assessment
    quality_level: QualityLevel = QualityLevel.GOOD
    quality_score: float = 1.0
    issues: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    timestamp: float = 0.0


@dataclass
class LocalizationMetrics:
    """Metrics for localization quality (Phase 3 - to be implemented)."""
    pass


@dataclass
class MapQualityReport:
    """Comprehensive map quality report."""

    timestamp: float
    scan_quality: Optional[ScanQualityMetrics] = None
    map_structure: Optional[MapStructureMetrics] = None
    localization: Optional[LocalizationMetrics] = None

    overall_quality_level: QualityLevel = QualityLevel.GOOD
    overall_quality_score: float = 1.0
    summary: str = ""


class MapQualityAnalyzer:
    """
    Analyzes map quality across multiple dimensions.

    Phase 1: Scan Processing Quality
    - Point count and density
    - Angular coverage and gaps
    - Range validity
    - Scan rate consistency
    """

    def __init__(
        self,
        min_point_count: int = 100,
        min_scan_density: float = 0.5,
        min_angular_coverage: float = 0.7,
        max_allowed_gap: float = 45.0,
        expected_scan_rate: float = 10.0,
        scan_rate_tolerance: float = 0.2
    ):
        """
        Initialize map quality analyzer.

        Args:
            min_point_count: Minimum acceptable points per scan
            min_scan_density: Minimum acceptable points per degree
            min_angular_coverage: Minimum required angular coverage (0-1)
            max_allowed_gap: Maximum allowed gap in degrees
            expected_scan_rate: Expected scan rate in Hz
            scan_rate_tolerance: Tolerance for scan rate variation (0-1)
        """
        self.min_point_count = min_point_count
        self.min_scan_density = min_scan_density
        self.min_angular_coverage = min_angular_coverage
        self.max_allowed_gap = max_allowed_gap
        self.expected_scan_rate = expected_scan_rate
        self.scan_rate_tolerance = scan_rate_tolerance

        # History tracking
        self.scan_history: List[ScanQualityMetrics] = []
        self.max_history_size = 100
        self.last_scan_time = 0.0

    def analyze_scan_quality(
        self,
        ranges: List[float],
        angles: Optional[List[float]] = None,
        angle_min: float = 0.0,
        angle_max: float = 2 * np.pi,
        angle_increment: Optional[float] = None,
        range_min: float = 0.1,
        range_max: float = 30.0
    ) -> ScanQualityMetrics:
        """
        Analyze quality of a laser scan.

        Args:
            ranges: List of range measurements
            angles: Optional list of angles (if not uniform)
            angle_min: Minimum angle (radians)
            angle_max: Maximum angle (radians)
            angle_increment: Angular increment (radians)
            range_min: Minimum valid range
            range_max: Maximum valid range

        Returns:
            ScanQualityMetrics with detailed quality assessment
        """
        metrics = ScanQualityMetrics()
        metrics.timestamp = time.time()

        if not ranges:
            metrics.quality_level = QualityLevel.CRITICAL
            metrics.quality_score = 0.0
            metrics.issues.append("No scan data received")
            return metrics

        # Calculate angle increment if not provided
        if angle_increment is None:
            angle_increment = (angle_max - angle_min) / len(ranges)

        # Generate angles if not provided
        if angles is None:
            angles = [angle_min + i * angle_increment for i in range(len(ranges))]

        # Convert to numpy for efficient computation
        ranges_arr = np.array(ranges)
        angles_arr = np.array(angles)

        # Filter valid ranges
        valid_mask = (ranges_arr >= range_min) & (ranges_arr <= range_max) & np.isfinite(ranges_arr)
        valid_ranges = ranges_arr[valid_mask]
        valid_angles = angles_arr[valid_mask]

        # Point count metrics
        metrics.point_count = len(valid_ranges)
        metrics.expected_point_count = len(ranges)

        if metrics.point_count < self.min_point_count:
            metrics.issues.append(
                f"Low point count: {metrics.point_count} < {self.min_point_count}"
            )
        elif metrics.point_count < metrics.expected_point_count * 0.5:
            metrics.warnings.append(
                f"Point count below 50% of expected: {metrics.point_count}/{metrics.expected_point_count}"
            )

        # Scan density metrics
        if len(valid_angles) > 0:
            angular_span = float(np.max(valid_angles) - np.min(valid_angles))
            if angular_span > 0:
                metrics.scan_density = len(valid_angles) / np.degrees(angular_span)

            if metrics.scan_density < self.min_scan_density:
                metrics.issues.append(
                    f"Low scan density: {metrics.scan_density:.2f} < {self.min_scan_density}"
                )

        # Angular coverage analysis
        total_angular_range = angle_max - angle_min
        if len(valid_angles) > 0:
            # Calculate coverage by checking angular bins
            num_bins = 360
            bin_size = 2 * np.pi / num_bins
            bins = np.zeros(num_bins, dtype=bool)

            for angle in valid_angles:
                # Normalize angle to [0, 2π)
                normalized_angle = (angle - angle_min) % (2 * np.pi)
                bin_idx = int(normalized_angle / bin_size) % num_bins
                bins[bin_idx] = True

            metrics.angular_coverage = float(np.sum(bins)) / num_bins

            # Find gaps
            gaps = []
            in_gap = False
            gap_start = 0

            for i in range(num_bins):
                if not bins[i]:
                    if not in_gap:
                        gap_start = i
                        in_gap = True
                else:
                    if in_gap:
                        gap_size = (i - gap_start) * np.degrees(bin_size)
                        gaps.append((
                            gap_start * np.degrees(bin_size),
                            i * np.degrees(bin_size)
                        ))
                        metrics.max_gap_size = max(metrics.max_gap_size, gap_size)
                        in_gap = False

            # Check if gap wraps around
            if in_gap:
                gap_size = (num_bins - gap_start) * np.degrees(bin_size)
                gaps.append((gap_start * np.degrees(bin_size), 360.0))
                metrics.max_gap_size = max(metrics.max_gap_size, gap_size)

            metrics.coverage_gaps = gaps

            if metrics.angular_coverage < self.min_angular_coverage:
                metrics.issues.append(
                    f"Low angular coverage: {metrics.angular_coverage:.1%} < {self.min_angular_coverage:.1%}"
                )

            if metrics.max_gap_size > self.max_allowed_gap:
                metrics.warnings.append(
                    f"Large coverage gap detected: {metrics.max_gap_size:.1f}° > {self.max_allowed_gap}°"
                )

        # Range quality metrics
        if len(valid_ranges) > 0:
            metrics.mean_range = float(np.mean(valid_ranges))
            metrics.median_range = float(np.median(valid_ranges))
            metrics.range_std = float(np.std(valid_ranges))
            metrics.valid_range_ratio = len(valid_ranges) / len(ranges)

            if metrics.valid_range_ratio < 0.5:
                metrics.warnings.append(
                    f"Low valid range ratio: {metrics.valid_range_ratio:.1%}"
                )

        # Scan rate metrics
        if self.last_scan_time > 0:
            time_delta = metrics.timestamp - self.last_scan_time
            if time_delta > 0:
                metrics.scan_rate = 1.0 / time_delta

                rate_error = abs(metrics.scan_rate - self.expected_scan_rate) / self.expected_scan_rate
                if rate_error > self.scan_rate_tolerance:
                    metrics.warnings.append(
                        f"Scan rate deviation: {metrics.scan_rate:.1f}Hz (expected {self.expected_scan_rate}Hz)"
                    )

        self.last_scan_time = metrics.timestamp

        # Calculate overall quality score and level
        metrics.quality_score, metrics.quality_level = self._calculate_scan_quality(metrics)

        # Store in history
        self.scan_history.append(metrics)
        if len(self.scan_history) > self.max_history_size:
            self.scan_history.pop(0)

        return metrics

    def _calculate_scan_quality(self, metrics: ScanQualityMetrics) -> tuple[float, QualityLevel]:
        """
        Calculate overall quality score and level from metrics.

        Returns:
            Tuple of (score, level) where score is 0-1
        """
        score = 1.0

        # Point count contribution (30%)
        if metrics.point_count >= metrics.expected_point_count * 0.8:
            point_score = 1.0
        elif metrics.point_count >= self.min_point_count:
            point_score = 0.5 + 0.5 * (
                (metrics.point_count - self.min_point_count) /
                (metrics.expected_point_count * 0.8 - self.min_point_count)
            )
        else:
            point_score = max(0.0, metrics.point_count / self.min_point_count) * 0.5

        # Coverage contribution (30%)
        if metrics.angular_coverage >= 0.95:
            coverage_score = 1.0
        elif metrics.angular_coverage >= self.min_angular_coverage:
            coverage_score = 0.5 + 0.5 * (
                (metrics.angular_coverage - self.min_angular_coverage) /
                (0.95 - self.min_angular_coverage)
            )
        else:
            coverage_score = max(0.0, metrics.angular_coverage / self.min_angular_coverage) * 0.5

        # Density contribution (20%)
        if metrics.scan_density >= metrics.expected_scan_density:
            density_score = 1.0
        elif metrics.scan_density >= self.min_scan_density:
            density_score = 0.5 + 0.5 * (
                (metrics.scan_density - self.min_scan_density) /
                (metrics.expected_scan_density - self.min_scan_density)
            )
        else:
            density_score = max(0.0, metrics.scan_density / self.min_scan_density) * 0.5

        # Valid range ratio contribution (20%)
        range_score = metrics.valid_range_ratio

        # Weighted combination
        score = (
            0.30 * point_score +
            0.30 * coverage_score +
            0.20 * density_score +
            0.20 * range_score
        )

        # Apply penalties for critical issues
        if len(metrics.issues) > 0:
            score *= 0.8
        if len(metrics.issues) > 2:
            score *= 0.7

        # Determine quality level
        if score >= 0.9:
            level = QualityLevel.EXCELLENT
        elif score >= 0.75:
            level = QualityLevel.GOOD
        elif score >= 0.5:
            level = QualityLevel.FAIR
        elif score >= 0.3:
            level = QualityLevel.POOR
        else:
            level = QualityLevel.CRITICAL

        return score, level

    def get_quality_summary(self) -> Dict[str, Any]:
        """
        Get summary of recent scan quality.

        Returns:
            Dictionary with quality statistics
        """
        if not self.scan_history:
            return {
                "status": "no_data",
                "message": "No scan data available"
            }

        recent_scans = self.scan_history[-10:]  # Last 10 scans

        avg_score = np.mean([s.quality_score for s in recent_scans])
        avg_points = np.mean([s.point_count for s in recent_scans])
        avg_coverage = np.mean([s.angular_coverage for s in recent_scans])
        avg_density = np.mean([s.scan_density for s in recent_scans])

        # Count issues
        total_issues = sum(len(s.issues) for s in recent_scans)
        total_warnings = sum(len(s.warnings) for s in recent_scans)

        # Get most recent quality level
        latest_level = recent_scans[-1].quality_level

        return {
            "status": "ok",
            "latest_quality_level": latest_level.value,
            "average_quality_score": float(avg_score),
            "average_point_count": float(avg_points),
            "average_angular_coverage": float(avg_coverage),
            "average_scan_density": float(avg_density),
            "total_issues": total_issues,
            "total_warnings": total_warnings,
            "scans_analyzed": len(recent_scans),
            "timestamp": time.time()
        }

    def analyze_map_structure(
        self,
        map_data: Dict[str, Any]
    ) -> MapStructureMetrics:
        """
        Analyze map structure quality (Phase 2).

        Args:
            map_data: Map message from SLAM containing width, height, resolution, data

        Returns:
            MapStructureMetrics with detailed map quality analysis
        """
        metrics = MapStructureMetrics()
        metrics.timestamp = time.time()

        try:
            width = map_data.get('width', 0)
            height = map_data.get('height', 0)
            resolution = map_data.get('resolution', 0.05)
            data = map_data.get('data', [])

            if not data or width == 0 or height == 0:
                metrics.is_empty = True
                metrics.quality_level = QualityLevel.CRITICAL
                metrics.quality_score = 0.0
                metrics.issues.append("Map is empty or invalid")
                return metrics

            # Basic map info
            metrics.map_width = width
            metrics.map_height = height
            metrics.resolution = resolution
            metrics.total_cells = width * height

            # Convert to numpy array for efficient processing
            map_array = np.array(data, dtype=np.int8).reshape(height, width)

            # Occupancy distribution
            occupied_mask = map_array == 100
            free_mask = map_array == 0
            unknown_mask = map_array == -1

            metrics.occupied_cells = int(np.sum(occupied_mask))
            metrics.free_cells = int(np.sum(free_mask))
            metrics.unknown_cells = int(np.sum(unknown_mask))

            known_cells = metrics.occupied_cells + metrics.free_cells
            if known_cells > 0:
                metrics.occupied_ratio = metrics.occupied_cells / known_cells
                metrics.free_ratio = metrics.free_cells / known_cells

            if metrics.total_cells > 0:
                metrics.unknown_ratio = metrics.unknown_cells / metrics.total_cells
                metrics.explored_ratio = known_cells / metrics.total_cells

            # Check if map has any content
            metrics.is_empty = (known_cells < 100)  # Less than 100 known cells = empty
            metrics.has_walls = (metrics.occupied_cells > 10)

            if metrics.is_empty:
                metrics.quality_level = QualityLevel.POOR
                metrics.quality_score = 0.2
                metrics.issues.append("Map is mostly unexplored")
                return metrics

            # Calculate entropy (measure of map information content)
            if known_cells > 0:
                p_occupied = metrics.occupied_cells / known_cells if known_cells > 0 else 0
                p_free = metrics.free_cells / known_cells if known_cells > 0 else 0

                # Shannon entropy
                entropy_terms = []
                if p_occupied > 0:
                    entropy_terms.append(-p_occupied * np.log2(p_occupied))
                if p_free > 0:
                    entropy_terms.append(-p_free * np.log2(p_free))

                metrics.entropy = float(np.sum(entropy_terms))

            # Noise detection: count isolated occupied cells
            if metrics.occupied_cells > 0:
                # Simple noise metric: occupied cells with no occupied neighbors
                kernel = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]], dtype=np.uint8)

                from scipy import ndimage
                # Count neighbors for each occupied cell
                neighbor_count = ndimage.convolve(occupied_mask.astype(np.uint8), kernel, mode='constant')

                # Isolated cells have 0 occupied neighbors
                isolated_occupied = occupied_mask & (neighbor_count == 0)
                isolated_count = np.sum(isolated_occupied)

                metrics.noise_score = isolated_count / metrics.occupied_cells if metrics.occupied_cells > 0 else 0.0

                if metrics.noise_score > 0.1:  # More than 10% noise
                    metrics.warnings.append(f"High noise level: {metrics.noise_score:.1%} isolated cells")

            # Wall sharpness: measure gradient magnitude at occupied cells
            if metrics.has_walls:
                # Convert to float for gradient calculation
                map_float = map_array.astype(np.float32)
                map_float[unknown_mask] = 50  # Unknown = neutral value

                # Calculate gradients
                grad_y, grad_x = np.gradient(map_float)
                gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2)

                # Average gradient at occupied cells (higher = sharper walls)
                wall_gradients = gradient_magnitude[occupied_mask]
                if len(wall_gradients) > 0:
                    metrics.wall_sharpness = float(np.mean(wall_gradients))

                # Wall sharpness assessment
                if metrics.wall_sharpness < 30:
                    metrics.warnings.append(f"Blurry walls detected (sharpness: {metrics.wall_sharpness:.1f})")

            # Feature detection (corners and edges)
            if metrics.has_walls and known_cells > 1000:  # Only for reasonably sized maps
                try:
                    # Prepare map for feature detection (binary: occupied = 255, rest = 0)
                    binary_map = np.zeros_like(map_array, dtype=np.uint8)
                    binary_map[occupied_mask] = 255

                    # Simple corner detection using Harris corner detector (lightweight)
                    from scipy import ndimage

                    # Sobel edges
                    edges_x = ndimage.sobel(binary_map, axis=1)
                    edges_y = ndimage.sobel(binary_map, axis=0)
                    edge_magnitude = np.sqrt(edges_x**2 + edges_y**2)

                    # Count edges (threshold at 50% of max)
                    edge_threshold = np.max(edge_magnitude) * 0.5 if np.max(edge_magnitude) > 0 else 0
                    metrics.edge_count = int(np.sum(edge_magnitude > edge_threshold))

                    # Simple corner detection: look for high gradient intersections
                    corner_response = edges_x * edges_y
                    corner_threshold = np.max(np.abs(corner_response)) * 0.5 if np.max(np.abs(corner_response)) > 0 else 0
                    metrics.corner_count = int(np.sum(np.abs(corner_response) > corner_threshold))

                    # Feature density (features per square meter)
                    map_area_m2 = (width * height * resolution * resolution)
                    total_features = metrics.corner_count + metrics.edge_count
                    metrics.feature_density = total_features / map_area_m2 if map_area_m2 > 0 else 0.0

                except Exception as e:
                    log.debug(f"Feature detection failed: {e}")

            # Calculate overall quality score
            metrics.quality_score, metrics.quality_level = self._calculate_map_quality(metrics)

        except Exception as e:
            log.error(f"Map structure analysis failed: {e}", exc_info=True)
            metrics.quality_level = QualityLevel.CRITICAL
            metrics.quality_score = 0.0
            metrics.issues.append(f"Analysis error: {str(e)}")

        return metrics

    def _calculate_map_quality(self, metrics: MapStructureMetrics) -> tuple[float, QualityLevel]:
        """
        Calculate overall map quality score and level.

        Returns:
            Tuple of (score, level) where score is 0-1
        """
        if metrics.is_empty:
            return 0.2, QualityLevel.POOR

        score = 1.0

        # Exploration contribution (30%)
        if metrics.explored_ratio >= 0.05:  # At least 5% explored
            exploration_score = min(1.0, metrics.explored_ratio / 0.2)  # Full score at 20% explored
        else:
            exploration_score = 0.5

        # Entropy contribution (20%) - higher entropy = more varied/useful map
        if metrics.entropy > 0:
            entropy_score = min(1.0, metrics.entropy / 1.0)  # Max entropy for binary is 1.0
        else:
            entropy_score = 0.5

        # Noise contribution (20%) - lower noise = better
        noise_score = 1.0 - min(1.0, metrics.noise_score * 5)  # Penalize noise heavily

        # Feature density contribution (15%)
        if metrics.feature_density > 0:
            feature_score = min(1.0, metrics.feature_density / 100)  # Full score at 100 features/m²
        else:
            feature_score = 0.3

        # Wall sharpness contribution (15%)
        if metrics.wall_sharpness > 0:
            sharpness_score = min(1.0, metrics.wall_sharpness / 50)  # Full score at gradient=50
        else:
            sharpness_score = 0.5

        # Weighted combination
        score = (
            0.30 * exploration_score +
            0.20 * entropy_score +
            0.20 * noise_score +
            0.15 * feature_score +
            0.15 * sharpness_score
        )

        # Apply penalties for issues
        if len(metrics.issues) > 0:
            score *= 0.8
        if len(metrics.issues) > 2:
            score *= 0.7

        # Determine quality level
        if score >= 0.9:
            level = QualityLevel.EXCELLENT
        elif score >= 0.75:
            level = QualityLevel.GOOD
        elif score >= 0.5:
            level = QualityLevel.FAIR
        elif score >= 0.3:
            level = QualityLevel.POOR
        else:
            level = QualityLevel.CRITICAL

        return score, level

    def generate_report(self) -> MapQualityReport:
        """
        Generate comprehensive quality report.

        Returns:
            MapQualityReport with all available metrics
        """
        report = MapQualityReport(timestamp=time.time())

        if self.scan_history:
            latest_scan = self.scan_history[-1]
            report.scan_quality = latest_scan
            report.overall_quality_level = latest_scan.quality_level
            report.overall_quality_score = latest_scan.quality_score

            # Generate summary
            summary_parts = [
                f"Scan Quality: {latest_scan.quality_level.value.upper()}",
                f"Score: {latest_scan.quality_score:.2f}",
                f"Points: {latest_scan.point_count}",
                f"Coverage: {latest_scan.angular_coverage:.1%}",
            ]

            if latest_scan.issues:
                summary_parts.append(f"Issues: {len(latest_scan.issues)}")
            if latest_scan.warnings:
                summary_parts.append(f"Warnings: {len(latest_scan.warnings)}")

            report.summary = " | ".join(summary_parts)
        else:
            report.summary = "No scan data available"

        return report


# Default analyzer instance
default_analyzer = MapQualityAnalyzer(
    min_point_count=100,
    min_scan_density=0.5,
    min_angular_coverage=0.7,
    max_allowed_gap=45.0,
    expected_scan_rate=10.0,
    scan_rate_tolerance=0.2
)
