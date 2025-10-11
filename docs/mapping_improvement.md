# SLAM Mapping Quality Improvement Guide

**Status**: Planning/Tuning Phase
**Last Updated**: 2025-10-11
**Target**: Improve map accuracy, reduce drift, optimize scan matching

---

## Current Configuration Analysis

### Existing slam_toolbox Settings

Our current configuration (`ros2_docker/config/slam_toolbox_params.yaml`) has been tuned for **CPU efficiency on Raspberry Pi 5**, but this comes at the cost of mapping accuracy:

```yaml
# Current settings optimized for low CPU usage
minimum_time_interval: 0.15          # 6.7Hz scan processing
scan_buffer_size: 5                  # Only 5 scans buffered
map_update_interval: 3.0             # Map updates every 3 seconds
transform_publish_period: 0.1        # 10Hz TF updates

# Motion filtering (currently disabled)
minimum_travel_distance: 0.0         # Disabled (no encoder trust)
minimum_travel_heading: 0.0          # Disabled

# Loop closure
loop_search_maximum_distance: 2.0    # Conservative 2m search
loop_match_minimum_chain_size: 15    # Strict matching (less closures)
```

### Identified Issues

**1. Scan Processing Quality**
- **6.7Hz processing** (`minimum_time_interval: 0.15`) is too slow
- When driving at normal speed, scans are spaced ~15cm apart
- Fast rotations can miss details or cause misalignment
- Small `scan_buffer_size: 5` limits scan matching quality

**2. Stationary Noise**
- `minimum_travel_distance: 0.0` means SLAM processes scans even when robot is completely still
- Stationary scans add noise to the map (sensor jitter, small IMU drift)
- Wastes CPU cycles without improving map

**3. Loop Closure Opportunities**
- `loop_search_maximum_distance: 2.0m` may miss larger room loops
- `loop_match_minimum_chain_size: 15` is too strict (requires 15 consecutive good matches)
- Missing loop closures → accumulated drift never gets corrected

**4. Scan Matching Search Space**
- `correlation_search_space_dimension: 0.5` may be too narrow
- If robot moves faster than expected, scan matcher might fail to align properly

**5. Hardware Quirks**
- **LIDAR mounted backwards** (180° rotation in TF, corrected)
- **IMU mounted upside down** (quaternion remapped in `imu_bridge.py`)
- These corrections work, but any additional drift compounds errors

---

## Proposed Parameter Improvements

### Phase 1: Restore Scan Processing Quality

**Goal**: Improve scan alignment without overwhelming the Pi 5

```yaml
# Increase scan processing frequency
minimum_time_interval: 0.10          # 10Hz (was 6.7Hz)
# Rationale: Better temporal resolution, catches fast movements

# Increase scan buffer for better matching
scan_buffer_size: 10                 # (was 5)
# Rationale: More scans = more context for alignment

# Fresher map updates
map_update_interval: 2.0             # (was 3.0)
# Rationale: Dashboard sees changes faster, better for debugging
```

**Trade-off**: ~20% increase in CPU load, but Pi 5 should handle it fine.

---

### Phase 2: Enable Smart Motion Filtering

**Goal**: Reduce noise from stationary robot, save CPU when not moving

```yaml
# Filter out tiny movements (matches map resolution)
minimum_travel_distance: 0.05        # 5cm (was 0.0)
# Rationale: Same as map resolution, prevents processing jitter

# Filter out tiny rotations
minimum_travel_heading: 0.0175       # ~1 degree (was 0.0)
# Rationale: Sub-degree rotations don't add useful information

# Keep scan matching enabled
use_scan_matching: true
# Rationale: LIDAR-based motion detection is more reliable than encoders
```

**Trade-off**: Might skip scans during very slow creep movements, but that's usually when precision matters least.

---

### Phase 3: Improve Loop Closure Detection

**Goal**: Catch loops reliably to correct accumulated drift

```yaml
# Search wider area for loop candidates
loop_search_maximum_distance: 3.0    # (was 2.0)
# Rationale: Larger rooms need wider search

# Be less strict about chain size
loop_match_minimum_chain_size: 10    # (was 15)
# Rationale: 10 consecutive matches is still reliable

# Keep existing response thresholds (they're reasonable)
loop_match_minimum_response_coarse: 0.35
loop_match_minimum_response_fine: 0.45
```

**Trade-off**: More loop closure attempts = slightly more CPU, but worth it for accuracy.

---

### Phase 4: Widen Scan Matching Search (If Still Having Issues)

**Goal**: Handle faster movements or larger drift between scans

```yaml
# Increase search space for scan alignment
correlation_search_space_dimension: 0.8  # (was 0.5)
# Rationale: If robot moves >50cm between scans, need wider search

# Keep existing resolution (fine-grained enough)
correlation_search_space_resolution: 0.01
correlation_search_space_smear_deviation: 0.1
```

**Trade-off**: Wider search = more computation per scan, but improves robustness.

---

## Mapping Best Practices

### Driving Technique

**Speed**:
- Use **creep mode** for mapping (0.7 forward, 0.8 turn)
- Avoid sudden accelerations or stops
- Smooth, constant velocity gives best scan alignment

**Rotation**:
- Turn slowly and smoothly
- Avoid spinning in place (scan matcher can lose track)
- If you must spin, do it very slowly (<0.3 rad/s)

**Coverage**:
- Drive close to walls first (establishes boundaries)
- Then fill in the middle
- Overlap your paths (helps loop closure)

### Loop Closure Strategy

**Why loops matter**:
- SLAM accumulates small errors over time (drift)
- Loop closure detects "I've been here before" and corrects the drift
- Without loops, your map slowly distorts

**How to close loops**:
1. Map a room by driving around the perimeter
2. Return to your starting point via the **same path** (at least partially)
3. Watch for loop closure events in ROS logs: `[slam_toolbox] Loop closed!`
4. If no loop detected, drive the overlap again more slowly

**Frequency**:
- Small room: Close loop every 3-5 meters of travel
- Large area: Close loop every 10 meters max
- If map looks distorted, drive back to a known-good area immediately

### Environment Considerations

**Good for SLAM**:
- Structured indoor environments (walls, furniture)
- Static objects (non-moving obstacles)
- Varied geometry (corners, doorways help with alignment)

**Problematic**:
- Long, featureless hallways (nothing to align to)
- Highly dynamic environments (people walking around)
- Reflective surfaces (glass, mirrors - LIDAR sees through them)
- Very tight spaces (robot might hit obstacles before LIDAR sees them)

**Lighting**: SLAM uses LIDAR only, so lighting doesn't matter (unless you add camera later)

---

## Implementation Guide

### Step 1: Backup Current Config

```bash
cd /home/keith/gordonbot/ros2_docker/config
cp slam_toolbox_params.yaml slam_toolbox_params.yaml.backup
```

### Step 2: Apply Phase 1 Changes

Edit `slam_toolbox_params.yaml`:

```yaml
minimum_time_interval: 0.10
scan_buffer_size: 10
map_update_interval: 2.0
```

### Step 3: Restart ROS Container

```bash
cd /home/keith/gordonbot/ros2_docker
sudo docker compose restart
```

Wait ~5 seconds for slam_toolbox to initialize.

### Step 4: Clear Map and Test

In the dashboard:
1. Click "Clear Map" button
2. Wait for confirmation
3. Drive robot in a **simple rectangular path** (2m × 2m)
4. Return to starting point to trigger loop closure

### Step 5: Evaluate Results

**Look for**:
- **Wall sharpness**: Are walls crisp or blurry?
- **Corner accuracy**: Are 90° corners actually 90°?
- **Loop closure**: Did returning to start correct any drift?
- **CPU usage**: Check with `docker stats gordonbot-ros2-slam`

**Good map characteristics**:
- Straight walls appear as single-pixel lines
- Parallel walls are actually parallel
- Rooms have correct dimensions
- No "ghosting" (duplicate walls offset slightly)

### Step 6: Iterate

If map still has issues:
1. Apply Phase 2 (motion filtering)
2. Apply Phase 3 (loop closure)
3. If still struggling, apply Phase 4 (wider search)

If CPU usage is too high:
- Reduce `scan_buffer_size: 10 → 8`
- Increase `minimum_time_interval: 0.10 → 0.12`

---

## Testing & Validation

### Metrics to Track

**Quantitative**:
- CPU usage (via `docker stats`): Target <50% on Pi 5
- Scan processing rate (via ROS logs): Should match `1/minimum_time_interval`
- Loop closure frequency: Count `Loop closed!` messages
- Map update latency: Time from robot move to dashboard update

**Qualitative**:
- Wall sharpness (visual inspection)
- Geometric accuracy (measure real room vs. map dimensions)
- Drift accumulation (drive 10m loop, check if end matches start)
- Consistency (remap same area twice, compare results)

### Test Scenarios

**Scenario 1: Rectangle Test**
- Map 2m × 3m rectangular area
- Close loop by returning to start
- Measure corner angles (should be 90°)
- Measure wall lengths (should match real dimensions)

**Scenario 2: Long Corridor**
- Map 10m straight hallway
- Drive to end, return to start
- Check for wall waviness or doubling
- Verify loop closure corrected any drift

**Scenario 3: Multi-Room**
- Map 2-3 connected rooms
- Close loops by re-entering previous rooms
- Check doorway alignment between rooms
- Verify no "map tears" at doorways

**Scenario 4: Fast Movement**
- Drive at normal speed (not creep mode)
- Test if scan matching keeps up
- Look for motion blur or misalignment

---

## Troubleshooting Common Issues

### Issue: Walls appear doubled or blurry

**Cause**: Scan misalignment, poor temporal registration

**Solutions**:
1. Increase `scan_buffer_size` (more context)
2. Decrease `minimum_time_interval` (finer temporal resolution)
3. Drive more slowly during mapping
4. Check IMU calibration (bad orientation → bad alignment)

---

### Issue: Map drifts over time (rooms distort)

**Cause**: No loop closures, accumulated odometry error

**Solutions**:
1. Increase `loop_search_maximum_distance` (find loops farther away)
2. Decrease `loop_match_minimum_chain_size` (be less strict)
3. Actively close loops by returning to mapped areas
4. Check wheel encoders (if enabled) for slippage

---

### Issue: SLAM "loses track" during fast rotations

**Cause**: Scan matcher can't find alignment between consecutive scans

**Solutions**:
1. Increase `correlation_search_space_dimension` (wider search)
2. Reduce rotation speed (slower than 0.5 rad/s recommended)
3. Increase `scan_buffer_size` (helps recover from temporary loss)
4. Add rotational constraints if using IMU

---

### Issue: CPU usage too high (>70% on Pi 5)

**Cause**: Too aggressive scan processing

**Solutions**:
1. Increase `minimum_time_interval` (process fewer scans)
2. Decrease `scan_buffer_size` (less computation per scan)
3. Increase `map_update_interval` (publish map less often)
4. Reduce `loop_search_maximum_distance` (smaller search area)

---

### Issue: Map has "ghost walls" (faint duplicate features)

**Cause**: Stationary robot processing sensor noise

**Solutions**:
1. Enable motion filtering (`minimum_travel_distance > 0`)
2. Increase `link_match_minimum_response_fine` (reject weak matches)
3. Check for vibration (robot shaking while stopped)
4. Verify IMU isn't drifting when stationary

---

### Issue: Loop closure never triggers

**Cause**: Search area too small or match threshold too strict

**Solutions**:
1. Increase `loop_search_maximum_distance: 2.0 → 3.0`
2. Decrease `loop_match_minimum_chain_size: 15 → 10`
3. Decrease `loop_match_minimum_response_coarse: 0.35 → 0.30`
4. Drive overlap path more slowly (gives SLAM more time)

---

## Expected Results After Tuning

### Before (Current Config)
- Walls: Blurry, 2-3 pixels thick
- Corners: Rounded, imprecise
- Loop closure: Infrequent, 2m range only
- Drift: Noticeable after 10m travel
- CPU: ~25-35% (underutilized)

### After (Optimized Config)
- Walls: Sharp, 1 pixel thick
- Corners: Crisp 90° angles
- Loop closure: Reliable, 3m range
- Drift: Minimal, corrected by loops
- CPU: ~40-50% (balanced)

### Performance Trade-offs
- **+15-20% CPU usage**: Worth it for accuracy
- **+10ms map update latency**: Negligible for human operator
- **Better real-time performance**: 10Hz processing vs 6.7Hz

---

## Future Enhancements

### Sensor Fusion Improvements
- Enable odometry-based motion filtering (when encoders are trustworthy)
- Tune IMU integration weights if angular drift is observed
- Add camera-based features for texture-rich environments (future)

### Advanced SLAM Features
- Multi-session mapping (merge maps from different runs)
- 3D mapping (if LIDAR supports Z-axis)
- Dynamic object filtering (ignore moving obstacles)
- Semantic labeling (door, wall, furniture detection)

### Automation
- Auto-tune parameters based on CPU usage and map quality metrics
- Real-time drift detection and correction
- Automatic loop closure verification

---

## References

- [slam_toolbox documentation](https://github.com/SteveMacenski/slam_toolbox)
- [slam_toolbox parameters guide](https://github.com/SteveMacenski/slam_toolbox#slam-toolbox-parameters)
- [ROS2 SLAM tuning guide](https://navigation.ros.org/tuning/index.html)
- GordonBot docs: `docs/slam.md`, `docs/slam_plan.md`

---

## Change Log

### 2025-10-11
- Initial documentation created
- Analyzed current configuration trade-offs
- Proposed phased improvement plan
- Added best practices and troubleshooting guide
