# artifacts_cvsa

This directory contains the artifact detection node. This node is responsible for the real-time identification of physiological artifacts (EOG and signal peaks) in the raw EEG signal.

---

### 1. Input

* **Topic:** `/neurodata`
* **Message Type:** `rosneuro_msgs/NeuroFrame`
* **Data:** The node subscribes to this topic, expecting to receive raw EEG signal samples in real-time.

---

### 2. Output

* **Topic:** `/cvsa/artifact_presence`
* **Message Type:** Custom (defined in this package)
* **Data:** The node publishes a custom message containing two fields:
    * `has_artifact` (bool): `true` if an artifact (EOG or peak) is detected, `false` otherwise.
    * `seq` (uint32): The sequence number (sample index) of the analyzed sample, used to synchronize this output with other processing nodes (e.g., `cvsa_processing`).

---

### 3. Configuration

This node **requires a YAML configuration file** that defines all the filters, channels, and thresholds necessary for detection.

The YAML file must contain the following fields:

* `sampleRate`: The sampling frequency of the input data (e.g., 512 Hz).
* `th_hEOG`: Threshold (in µV) for horizontal eye movement.
* `th_vEOG`: Threshold (in µV) for vertical eye movement.
* `th_peaks`: Threshold (in µV) for signal peaks (e.g., muscle artifacts or saturation).
* `EOG_ch`: A list of 3 integers, specifying the indices of the **Fp1**, **Fp2**, and **EOG** channels (in that order).
* `freq_high_EOG` / `freq_low_EOG`: Cutoff frequencies (in Hz) for the EOG band-pass filter (e.g., 1-10 Hz).
* `freq_high_peaks`: Cutoff frequency (in Hz) for the peaks high-pass filter (e.g., 2 Hz).
* `filterOrder_EOG` / `filterOrder_peaks`: The order of the respective IIR (Butterworth) filters.

#### Example `artifact_cfg.yaml`

```yaml
ArtifactCfg:
  name: artifact
  params: 
    sampleRate: 512
    th_hEOG: 70
    th_vEOG: 70
    th_peaks: 160
    EOG_ch: [1,2,19]
    freq_high_EOG: 1
    freq_low_EOG: 10
    freq_high_peaks: 2
    filterOrder_EOG: 4
    filterOrder_peaks: 4
```

---

### 4. Workflow

1.  **Load Configuration:** On startup, the node loads all parameters (thresholds, channels, filters) from the YAML file.
2.  **Buffer Data:** The node maintains a **ring buffer** of 512 samples (equivalent to 1 second of data at 512 Hz) for analysis.
3.  **EOG Monitoring:**
    * The EOG channels (defined in `EOG_ch`) are extracted.
    * A band-pass filter (e.g., 1-10 Hz) is applied to the EOG channels.
    * Horizontal movement is calculated: $hEOG = Fp1 - Fp2$.
    * Vertical movement is calculated: $vEOG = (Fp1 + Fp2) / 2 - EOG$.
    * If `abs(hEOG) > th_hEOG` OR `abs(vEOG) > th_vEOG`, the EOG artifact is considered present.
4.  **Peak Monitoring:**
    * A high-pass filter (e.g., 2 Hz) is applied to all EEG channels (to remove the problematic DC drift from the AntNeuro cap).
    * The absolute value of each filtered sample, on every channel, is compared against `th_peaks`.
    * If *any* channel exceeds the threshold (`abs(sample) > th_peaks`), the peak-type artifact is considered present.
5.  **Publish:**
    * If the EOG check OR the Peak check is positive, the node sets `has_artifact = true`.
    * Otherwise, it sets `has_artifact = false`.
    * The node publishes the message on `/cvsa/artifact_presence`, including the current sample's `seq` number.

---

### 5. Dependencies

This package requires several libraries for compilation and execution:

* **rosneuro_msgs:** Required for the `NeuroFrame` input message.
* **rosneuro_filters:** Used for the band-pass and high-pass filter implementations.
* **Eigen:** Required for linear algebra operations and vector management.
* **rtf (Ring-Time-Framework):** Used for the efficient real-time ring buffer implementation.
* **yaml-cpp:** Required for parsing the `.yaml` configuration file.