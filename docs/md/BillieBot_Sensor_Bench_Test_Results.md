# BillieBot Sensor Bench Test Report

## 1. Purpose

This report documents the results of the BillieBot sensor bench-test campaign and compares the measured results against the acceptance requirements defined in the **BillieBot Sensor Bench Test Plan**.

The report covers the following sensors and test classes for which results were provided:

* **Luxonis OAK-D Lite**

  * UT-OAK-01 — Basic RGB, Stereo Depth, and Point-Cloud Acquisition
  * UT-OAK-02 — Flat-Target Depth Accuracy and Precision
  * DT-OAK-01 — Live Dog Detection and Stereo Range
* **MLX90640 Thermal Camera**

  * UT-THM-01 — Thermal Frame Acquisition and Visualization
  * UT-THM-02 — Temperature Bias, Noise, and Contrast-to-Noise
  * DT-THM-01 — Warm-Body Blob Detection
* **reSpeaker XVF3800**

  * UT-AUD-01 — USB Audio Enumeration, WAV Capture, and Signal Quality
  * DT-AUD-01 — Bark, Speech, and Other-Sound Classification
  * DT-AUD-02 — Direction of Arrival

No Pi Camera Module 3 NoIR test results were included in the results supplied for this report; therefore, UT-NIR-01 and UT-NIR-02 are not evaluated here.

---

# 2. Executive Summary

## 2.1 Test Status Summary

| Test ID          | Sensor           | Test                         | Status                                             | Principal Finding                                                                                                                                                                                                                        |
| ---------------- | ---------------- | ---------------------------- | -------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **UT-OAK-01**    | OAK-D Lite       | Stream acquisition           | **PASS**                                           | RGB, depth, and point-cloud streams were present; RGB and depth operated at approximately 5 Hz with acceptable frame gaps.                                                                                                               |
| **UT-OAK-02**    | OAK-D Lite       | Depth accuracy               | **QUALIFIED PASS**                                 | Excellent depth accuracy and precision at 1.492 m; however, the test plan specifies 2.00 m as the formal acceptance distance. Plane-RMSE calculation was invalid because of an unusable first frame.                                     |
| **DT-OAK-01**    | OAK-D Lite       | Dog detection                | **SMOKE TEST SUCCESSFUL — FORMAL TEST INCOMPLETE** | Billie was successfully detected with valid 3-D detections and bounding boxes at approximately 5 Hz. The smoke-test positive fraction was 79.3%; controlled distance, negative-scene, and 2 m range-accuracy testing were not performed. |
| **UT-THM-01**    | MLX90640         | Thermal acquisition          | **PASS**                                           | Thermal data were continuous, 100% finite, and published at 3.932 Hz with exceptionally stable timing.                                                                                                                                   |
| **UT-THM-02**    | MLX90640         | Temperature characterization | **CHARACTERIZATION PASS**                          | Very strong thermal contrast and stable measurements. Reference uncertainty was 1.5 °C, making the run characterization-only under the test plan. The recorded target reference temperature also requires clarification.                 |
| **DT-THM-01**    | MLX90640         | Warm-body blob detection     | **FAIL / PARTIAL CHARACTERIZATION**                | Positive target was detected in 100% of frames at 0.5–1.5 m, but the empty baseline contained a 7.84% false-positive fraction. A black kettle rather than Billie was used as the target.                                                 |
| **UT-AUD-01**    | XVF3800          | Audio acquisition            | **FAIL**                                           | Ambient recording passed all supplied criteria, but speech and impulse recordings exceeded the 0.1% clipping requirement.                                                                                                                |
| **DT-AUD-01**    | XVF3800          | Bark classification          | **SMOKE TEST SUCCESSFUL — FORMAL TEST INCOMPLETE** | A live Billie bark produced high-confidence Animal/Dog YAMNet detections. Full recall, false-positive, latency, and sustained processing-rate tests were not performed.                                                                  |
| **DT-AUD-02**    | XVF3800          | Direction of arrival         | **DEFERRED**                                       | Test was not performed because the current room/corner configuration does not provide appropriate DoA geometry.                                                                                                                          |
| **UT-NIR-01/02** | Pi Camera 3 NoIR | Acquisition/image quality    | **NOT EVALUATED**                                  | No results were supplied for this report.                                                                                                                                                                                                |

## 2.2 Overall Assessment

The completed testing provides strong evidence that the **OAK-D Lite and MLX90640 acquisition chains are operating correctly**. UT-OAK-01 and UT-THM-01 both produced clear passing results, with stream rates closely matching their commanded rates and no evidence in the supplied metrics of major data-integrity problems.

The OAK-D depth characterization was also strong. At a measured range of 1.492 m, the median depth error was approximately **−4.3 cm**, corresponding to an approximately **2.9% range underestimate**, while 99.6% of valid pixels were within ±20 cm of the reference. The principal qualification is procedural: the formal test plan specifies **2.00 m** as the required acceptance distance, whereas the completed test was performed at 1.492 m.

Detector/classifier smoke tests successfully demonstrated both **visual dog detection** and **live bark-related audio classification**, but these runs did not contain the full controlled test matrices required to establish formal Type 2 acceptance.

Two areas require additional attention. First, DT-THM-01 failed its empty-baseline criterion because thermal blobs appeared in approximately 7.8% of baseline frames. Second, UT-AUD-01 failed because clipping exceeded the permitted 0.1% during speech and impulse recordings. Neither result indicates a fundamental sensor-interface failure, but both should be resolved before those tests are formally closed.

---

# 3. OAK-D Lite

## 3.1 UT-OAK-01 — Basic RGB, Stereo Depth, and Point-Cloud Acquisition

### Test Objective

Verify that the OAK-D Lite produces valid RGB, depth, and point-cloud data and that the requested approximately 5 Hz streams satisfy the required rate and continuity limits.

### Result

**Overall result: PASS**

### Acceptance Comparison

| Criterion                   |                 Measured Value | Requirement | Result                           |
| --------------------------- | -----------------------------: | ----------: | -------------------------------- |
| RGB stream present          |                           True |    Required | **PASS**                         |
| Depth stream present        |                           True |    Required | **PASS**                         |
| Point-cloud stream present  |                           True |    Required | **PASS**                         |
| RGB mean rate               |                        4.97 Hz |     ≥4.5 Hz | **PASS**                         |
| RGB maximum frame gap       |                        0.616 s |      ≤1.0 s | **PASS**                         |
| Depth mean rate             |                        4.97 Hz |     ≥4.5 Hz | **PASS**                         |
| Depth maximum frame gap     |                        0.616 s |      ≤1.0 s | **PASS**                         |
| Camera-info topics present  | Not reported in supplied notes |    Required | **Not independently documented** |
| Header timestamps monotonic | Not reported in supplied notes |    Required | **Not independently documented** |
| No device reset/disconnect  |              No reset reported |    Required | **No failure observed**          |

### Discussion

The measured RGB and depth publication rates of **4.97 Hz** are effectively equal to the intended 5 Hz bench rate and comfortably exceed the 4.5 Hz minimum acceptance threshold. The maximum observed gap of **0.616 s** is also well below the 1.0 s limit.

The available results therefore demonstrate reliable OAK-D RGB/depth acquisition and confirm that the principal high-bandwidth streams can be produced at the intended BillieBot bench rate.

The supplied automated bench report did not separately reproduce evidence for camera-info publication, timestamp monotonicity, or point-cloud publication rate. These are documentation limitations in the results summarized here rather than observed failures.

### Disposition

**PASS.**

The OAK-D Lite acquisition chain is considered successfully verified for the measured criteria.

---

## 3.2 UT-OAK-02 — Flat-Target Depth Accuracy and Precision

### Test Objective

Measure OAK-D range bias, precision, valid-pixel fraction, and planar depth performance against a target at a known range.

### Test Configuration

| Parameter                                    |                       Value |
| -------------------------------------------- | --------------------------: |
| Measured target distance                     |                 **1.492 m** |
| Target distance in inches                    |                    58.75 in |
| Target dimensions                            |             0.229 × 0.305 m |
| Target dimensions in inches                  |                   9 × 12 in |
| Raw depth resolution                         |                 1920 × 1080 |
| Foxglove depth preview                       |                   320 × 180 |
| Preview decimation                           |        6× in each dimension |
| Selected raw ROI                             | `(900,420)` to `(1092,660)` |
| Raw ROI dimensions                           |            192 × 240 pixels |
| Depth pixels per frame                       |                      46,080 |
| Approximate frames analyzed                  |                        ~279 |
| Approximate samples before invalid rejection |              ~12.86 million |

### Acceptance Comparison

| Criterion                  | Measured Value |         Requirement | Result                    |
| -------------------------- | -------------: | ------------------: | ------------------------- |
| Absolute median range bias |       ~0.043 m |             ≤0.20 m | **PASS**                  |
| Valid depth fraction       |          0.967 |               ≥0.90 | **PASS**                  |
| Fraction within ±0.20 m    |          0.996 |               ≥0.95 | **PASS**                  |
| Plane-fit RMSE             |            NaN | ≤0.05 m provisional | **Not validly evaluated** |
| Formal test distance       |        1.492 m |     2.00 m required | **Qualification**         |

### Depth Metrics

| Metric                          |                 Result |
| ------------------------------- | ---------------------: |
| Ground-truth distance           |                1.492 m |
| Median measured range           |                1.449 m |
| Mean measured range             |                1.455 m |
| Median bias                     | approximately −0.043 m |
| Robust standard deviation       |               0.0119 m |
| Conventional standard deviation |               0.0652 m |
| 5th percentile                  |                1.434 m |
| 50th percentile                 |                1.449 m |
| 95th percentile                 |                1.473 m |
| Valid-pixel fraction            |                 96.72% |
| Fraction within ±0.20 m         |                 99.56% |

### Discussion

The OAK-D exhibited very good depth performance during this run. The median measurement was approximately **4.3 cm shorter than the reference range**, corresponding to an approximately **2.9% underestimate** at 1.492 m. This is comfortably inside the BillieBot system requirement of ±20 cm.

Precision was also strong. The robust standard deviation was only **1.19 cm**, and the middle 90% of the valid depth population extended from **1.434 to 1.473 m**, a span of only **3.9 cm**.

The conventional standard deviation of approximately **6.52 cm** is substantially larger than the robust standard deviation. This indicates the presence of a relatively small number of outlying measurements rather than broad instability in the primary depth population. The robust statistic is therefore a more representative measure of the typical depth precision for this dataset.

The plane-fit metrics returned `NaN`. According to the test notes, this occurred because the plane-fitting routine used an invalid first frame. Consequently, the reported provisional plane-RMSE failure does **not** constitute evidence that the OAK-D exceeded the 5 cm plane-residual requirement; rather, the metric was not successfully calculated.

A second qualification is more important for formal test closure: the sensor test plan specifies **2.00 m as the required acceptance distance**, whereas this run was performed at **1.492 m**.

### Disposition

**QUALIFIED PASS.**

The sensor clearly satisfies the BillieBot ±0.20 m depth requirement at the tested 1.492 m range and demonstrates strong precision. For strict compliance with the test plan, a short repeat should eventually be performed with the target located at **2.00 m**, and the plane-fitting analysis should ignore invalid startup frames.

---

## 3.3 DT-OAK-01 — Live Dog Detection and Stereo Range

### Test Objective

Verify live dog detection using the production OAK-D detector, characterize detector recall and false positives, and confirm usable stereo range.

### Test Scope

This run was explicitly performed as a **smoke test rather than the full formal DT-OAK-01 campaign**.

### `/dog/found` Results

| Metric        |       Result |
| ------------- | -----------: |
| Messages      |          227 |
| Mean rate     | **4.975 Hz** |
| Maximum gap   |      0.582 s |
| True cycles   |          180 |
| False cycles  |           47 |
| True fraction |   **79.30%** |

### `/dog/detections_3d` Results

| Metric                      |   Result |
| --------------------------- | -------: |
| Detection messages          |      180 |
| All labels dog              |     True |
| Valid bounding-box fraction | **100%** |
| Median confidence           |    0.754 |
| Mean confidence             |    0.740 |
| Minimum confidence          |    0.502 |
| Maximum confidence          |    0.908 |
| Median depth                |  1.793 m |
| Mean depth                  |  1.826 m |
| Depth 5th percentile        |  1.622 m |
| Depth 95th percentile       |  2.129 m |

### Example Detections

| Detection | Confidence |   Depth | Bounding Box `(x,y,w,h)` |
| --------- | ---------: | ------: | ------------------------ |
| 1         |      0.779 | 1.892 m | `(167,96,102,107)`       |
| 2         |      0.593 | 1.793 m | `(170,100,100,103)`      |
| 3         |      0.529 | 1.793 m | `(168,95,127,109)`       |
| 4         |      0.531 | 1.793 m | `(163,94,130,111)`       |
| 5         |      0.667 | 1.793 m | `(165,99,116,106)`       |

### Acceptance Comparison

| Criterion                           |              Smoke-Test Result |     Requirement | Assessment                                   |
| ----------------------------------- | -----------------------------: | --------------: | -------------------------------------------- |
| `/dog/found` publication rate       |                       4.975 Hz |      4.5–5.5 Hz | **PASS**                                     |
| Aggregate dog recall                |              79.3% true cycles |   ≥85% at 1–4 m | **Below threshold if interpreted as recall** |
| 2.0 m median depth error            | No ground-truth range recorded |         ≤0.20 m | **Not evaluated**                            |
| Empty-scene false-positive fraction |            Not tested formally | ≤5% provisional | **Not evaluated**                            |
| Positive bounding boxes valid       |                           100% |        Required | **PASS**                                     |
| Min/max reliable distance           |                 Not determined | Report required | **Not evaluated**                            |

### Discussion

This smoke test successfully demonstrated the complete live OAK-D dog-detection chain. Billie produced **180 valid 3-D dog detections**, all reported bounding boxes were valid, and the `/dog/found` publication rate closely matched the required 5 Hz cadence.

Detector confidence was generally strong, with a median confidence of **0.754** and a maximum of **0.908**.

The 79.3% `/dog/found=true` fraction is below the formal test-plan recall requirement of 85%. However, this value should **not** be treated as the formal detector recall because the run did not use the required controlled distance/orientation segments and associated ground-truth protocol. It is best interpreted as a smoke-test duty fraction.

Similarly, although stereo depth measurements were produced successfully, no controlled ground-truth distance was associated with this recording; therefore, the formal ±0.20 m detector-depth requirement cannot be evaluated.

### Disposition

**SMOKE TEST SUCCESSFUL — FORMAL DT-OAK-01 INCOMPLETE.**

The test successfully demonstrated that Billie can be detected and spatially ranged by the production OAK-D pipeline. A full DT-OAK-01 campaign remains necessary if formal detector acceptance is desired.

---

# 4. MLX90640 Thermal Camera

## 4.1 UT-THM-01 — Thermal Frame Acquisition and Visualization

### Test Objective

Verify reliable 32 × 24 thermal-frame acquisition, approximately 4 Hz operation, and valid finite temperature data.

### Result

**Overall result: PASS**

### Acceptance Comparison

| Criterion             |                     Measured Value |      Requirement | Result                           |
| --------------------- | ---------------------------------: | ---------------: | -------------------------------- |
| Thermal image present |                               True |         Required | **PASS**                         |
| Finite-pixel fraction |                              1.000 |           ≥0.999 | **PASS**                         |
| Mean publication rate |                           3.932 Hz |       3.6–4.4 Hz | **PASS**                         |
| Dimensions/encoding   | Not reproduced in supplied results | 32 × 24, `32FC1` | **Not independently documented** |
| Sustained read errors |                      None reported |              <1% | **No failure reported**          |

### Timing Metrics

| Metric                    |          Value |
| ------------------------- | -------------: |
| Mean rate                 |  **3.9322 Hz** |
| Minimum period            |      0.25404 s |
| Maximum period            |      0.25509 s |
| Period standard deviation | **0.000120 s** |

### Discussion

The MLX90640 demonstrated excellent timing stability. At **3.932 Hz**, the measured rate is nearly ideal for the commanded 4 Hz operation and is comfortably inside the 3.6–4.4 Hz acceptance band.

The frame period varied by only approximately 1 ms between the observed minimum and maximum values, and the period standard deviation was approximately **0.12 ms**. In addition, the finite-pixel fraction was **100%**, indicating that no NaN or infinite temperature values were observed in the analyzed data.

### Disposition

**PASS.**

The MLX90640 acquisition path is considered successfully verified.

---

## 4.2 UT-THM-02 — Temperature Bias, Noise, and Contrast-to-Noise

### Test Objective

Characterize thermal target/background bias, contrast, temporal stability, and noise using controlled target and background regions.

### Test Configuration

| Parameter                             |                  Value |
| ------------------------------------- | ---------------------: |
| Recorded reference target temperature |                40.0 °C |
| Reference background temperature      |                26.2 °C |
| Reference uncertainty                 |            **±1.5 °C** |
| Camera-to-target distance             |                1.067 m |
| Target ROI                            | `(10,13)` to `(14,18)` |
| Background ROI                        |  `(14,1)` to `(30,10)` |
| Target ROI pixels                     |                     20 |
| Background ROI pixels                 |                    144 |
| Frames                                |                    233 |
| Duration                              |                59.04 s |
| Observed rate                         |               3.930 Hz |

### Measured Thermal Performance

| Metric                                     |       Result |
| ------------------------------------------ | -----------: |
| Target mean                                |     49.80 °C |
| Target median                              |     50.72 °C |
| Background mean                            |     28.03 °C |
| Background median                          |     28.00 °C |
| Target-background ΔT                       | **21.78 °C** |
| Recorded target bias                       |    +10.72 °C |
| Background bias                            |     +1.80 °C |
| CNR                                        |    **46.23** |
| Pooled CNR                                 |         4.56 |
| Mean target spatial standard deviation     |      4.74 °C |
| Mean background spatial standard deviation |     0.444 °C |
| Target temporal noise                      |    ~0.356 °C |
| Background temporal noise                  |    ~0.392 °C |
| Target drift                               |    −0.552 °C |
| Background drift                           |    −0.386 °C |
| NaN/Inf in ROI                             |         None |

### Acceptance Comparison

| Criterion             |                  Measured Value |                   Requirement | Result                    |
| --------------------- | ------------------------------: | ----------------------------: | ------------------------- |
| Target bias           | +10.72 °C using 40 °C reference |           ≤2.0 °C provisional | **FAIL numerically**      |
| Background bias       |                        +1.80 °C |           ≤2.0 °C provisional | **PASS**                  |
| CNR                   |                           46.23 |                            ≥5 | **PASS**                  |
| NaN/Inf in ROIs       |                            None |                  None allowed | **PASS**                  |
| Target drift          |              0.552 °C magnitude |                       ≤1.0 °C | **PASS**                  |
| Background drift      |              0.386 °C magnitude |                       ≤1.0 °C | **PASS**                  |
| Reference uncertainty |                          1.5 °C | >1 °C → characterization only | **CHARACTERIZATION ONLY** |

### Reference-Temperature Qualification

The test notes contain an important ambiguity regarding the target reference temperature: the operator notes state that the target may have been approximately **40 or 50 °C**, while the analysis file used **40.0 °C**.

This distinction materially changes the bias interpretation.

Using the recorded 40.0 °C reference:

[
50.716 - 40.0 = +10.716~^\circ\mathrm{C}
]

which fails the provisional ±2 °C criterion.

If the actual target reference were approximately 50.0 °C:

[
50.716 - 50.0 = +0.716~^\circ\mathrm{C}
]

which would pass comfortably.

Accordingly, the +10.72 °C result should not be interpreted as demonstrated MLX90640 calibration error until the actual reference-target temperature is confirmed.

### Discussion

Independent of absolute target accuracy, the test demonstrated very strong ability to distinguish the warm target from its background. The measured thermal separation was **21.78 °C**, and the reported background-noise-based CNR was **46.2**, nearly an order of magnitude greater than the provisional requirement of 5.

Temporal stability was also good. Target and background temporal noise were both approximately **0.4 °C**, and drift remained well within the 1 °C limit throughout the stationary capture.

The reference instrument uncertainty was **±1.5 °C**. The test plan explicitly states that when reference uncertainty exceeds 1 °C, the run is to be treated as **characterization only** and the sensor should not be failed solely because of the bias threshold.

### Disposition

**CHARACTERIZATION PASS — NOT A FORMAL CALIBRATION-ACCURACY PASS.**

The run successfully demonstrates good thermal contrast, low temporal noise, and acceptable drift. The actual target reference temperature should be verified before using this dataset to make an absolute temperature-accuracy claim.

---

## 4.3 DT-THM-01 — Warm-Body Blob Detection

### Test Objective

Verify the production thermal blob algorithm and characterize detection performance and false positives.

### Test Configuration

Four ground-truth segments were analyzed:

1. Empty baseline
2. Black kettle at 0.5 m
3. Black kettle at 1.0 m
4. Black kettle at 1.5 m

The run therefore exercised the thermal-blob algorithm using a **controlled warm object**, not Billie.

### Results

| Metric                                 |                         Result |
| -------------------------------------- | -----------------------------: |
| Total segments                         |                              4 |
| Total frames                           |                            491 |
| Total blobs                            |                            340 |
| Empty-baseline detection fraction      |                      **7.84%** |
| 0.5 m detection fraction               |                       **100%** |
| 1.0 m detection fraction               |                       **100%** |
| 1.5 m detection fraction               |                       **100%** |
| Valid positive-output fraction         |                           100% |
| Maximum demonstrated reliable distance |                      **1.5 m** |
| Mean blob area                         |                    58.5 pixels |
| Mean blob temperature                  |                       32.90 °C |
| Mean centroid                          | approximately `(10.39, 11.80)` |

### Acceptance Comparison

| Criterion                        |                             Measured Value |                        Requirement | Result            |
| -------------------------------- | -----------------------------------------: | ---------------------------------: | ----------------- |
| Empty baseline contains no blobs |                   7.84% detection fraction |                                 0% | **FAIL**          |
| Detection at 0.5 m               |                                       100% |                               ≥80% | **PASS**          |
| Detection at 1.0 m               |                                       100% |                               ≥80% | **PASS**          |
| Detection at 1.5 m               |                                       100% |                               ≥80% | **PASS**          |
| Positive outputs valid           |                                       100% |                           Required | **PASS**          |
| Area ≥8 pixels                   | Mean 58.5 px; scorer reports outputs valid |                           Required | **PASS**          |
| Mean temperature 30–40 °C        |                              Mean 32.90 °C |                           Required | **PASS**          |
| Billie used as positive target   |                     No — black kettle used | Billie required for formal DT test | **Not satisfied** |

### Discussion

The positive-target portion of this run was very strong. The warm target was detected in **100% of frames at all three tested distances**, demonstrating that the current global thresholding algorithm can robustly identify an appropriate thermal target out to at least **1.5 m**.

The formal test nevertheless failed because thermal blobs were produced during **7.84% of the empty-baseline frames**, whereas the acceptance requirement permits no blobs during the controlled empty baseline.

The test notes indicate that the operator suspects the false positives may have resulted from the test setup or ground-truth reconstruction. That possibility is plausible but cannot be established from the supplied metrics alone; therefore, the recorded run must retain its formal failure disposition.

The use of a black kettle is also important. This run is valuable for characterizing the thermal detection algorithm, but it does not establish that **Billie** produces the required ≥80% detection fraction at 0.5–1.5 m.

### Disposition

**FAIL for the recorded formal criteria; useful partial characterization completed.**

Recommended closure action is to repeat the test with:

* a carefully controlled empty baseline;
* Billie as the positive target;
* the same 0.5, 1.0, and 1.5 m distance conditions.

If the empty-baseline detections disappear on repeat, the current failure can reasonably be attributed to the original test condition rather than the detector itself.

---

# 5. reSpeaker XVF3800

## 5.1 UT-AUD-01 — Audio Acquisition and Signal Quality

### Test Objective

Verify valid 16 kHz, two-channel, 3 s audio acquisition and acceptable waveform integrity.

### Acceptance Requirements

* Sample rate: **16 kHz**
* Duration: **3.00 ±0.05 s**
* Clipping fraction: **<0.1%**, or `<0.001`
* Absolute normalized DC offset: **<0.01 full scale**, provisional

Three conditions were evaluated: ambient, speech, and impulse.

---

### 5.1.1 Ambient Recording

| Metric                      |    Result |
| --------------------------- | --------: |
| Sample rate                 | 16,000 Hz |
| Channels                    |         2 |
| Samples/channel             |    48,000 |
| Duration                    |    3.00 s |
| Channel 0 clipping fraction |         0 |
| Channel 1 clipping fraction |         0 |
| Channel correlation         |      0.97 |
| DC-offset criterion         |    Passed |

#### Acceptance

| Criterion                 | Result   |
| ------------------------- | -------- |
| Sample rate               | **PASS** |
| Duration                  | **PASS** |
| Clipping                  | **PASS** |
| DC offset                 | **PASS** |
| Overall ambient recording | **PASS** |

The quiet-room recording therefore demonstrated clean acquisition with no clipping and strong correlation between the two recorded channels.

---

### 5.1.2 Speech Recording

| Metric              |   Channel 0 | Channel 1 |
| ------------------- | ----------: | --------: |
| Clipping fraction   | **0.00785** |         0 |
| Clipping percentage |  **0.785%** |        0% |

Channel correlation was **0.83**.

#### Acceptance

| Criterion                | Result   |
| ------------------------ | -------- |
| Sample rate              | **PASS** |
| Duration                 | **PASS** |
| Clipping                 | **FAIL** |
| DC offset                | **PASS** |
| Overall speech recording | **FAIL** |

Channel 0 clipping was approximately **0.785%**, nearly eight times the maximum permitted 0.1%.

---

### 5.1.3 Impulse Recording

| Metric              |   Channel 0 |   Channel 1 |
| ------------------- | ----------: | ----------: |
| Clipping fraction   | **0.00375** | **0.00196** |
| Clipping percentage |  **0.375%** |  **0.196%** |

Channel correlation was **0.96**.

#### Acceptance

| Criterion                 | Result   |
| ------------------------- | -------- |
| Sample rate               | **PASS** |
| Duration                  | **PASS** |
| Clipping                  | **FAIL** |
| DC offset                 | **PASS** |
| Overall impulse recording | **FAIL** |

Both impulse channels exceeded the 0.1% clipping limit.

### Overall UT-AUD-01 Assessment

The XVF3800 successfully produced correctly timed, correctly sampled audio, and the ambient recording met all of the supplied quantitative acceptance criteria. The failure is specifically associated with **signal clipping under louder speech and impulse conditions**.

This demonstrates that the microphone interface and capture pipeline are operational, but the acoustic/input-gain configuration should be adjusted or the test stimulus level controlled before formal acceptance.

### Disposition

**FAIL.**

UT-AUD-01 should be repeated after addressing clipping. The repeat can be narrowly focused on speech and impulse conditions because the ambient acquisition already demonstrated acceptable sample rate, duration, and DC behavior.

---

## 5.2 DT-AUD-01 — Bark, Speech, and Other-Sound Classification

### Test Objective

Verify the production YAMNet classifier, including sustained processing cadence, bark recall, non-bark false-positive behavior, and event latency.

### Test Scope

Only a **live smoke test** was performed.

A naturally occurring Billie bark was successfully detected.

### Example Classifier Outputs

**Detection 1**

| Field        | Value      |
| ------------ | ---------- |
| Confidence   | **0.9922** |
| YAMNet label | `Animal`   |
| Energy       | −10.77 dB  |
| DoA          | 0.0°       |

**Detection 2**

| Field        | Value      |
| ------------ | ---------- |
| Confidence   | **0.4141** |
| YAMNet label | `Dog`      |
| DoA          | 0.0°       |

The two detections were separated by approximately **0.50 s**.

### Acceptance Comparison

| Criterion                          | Smoke-Test Evidence            | Requirement                 | Assessment                   |
| ---------------------------------- | ------------------------------ | --------------------------- | ---------------------------- |
| Bark-related classification occurs | Animal/Dog detections observed | Required                    | **Demonstrated**             |
| Processing-loop frequency          | Two events ~0.5 s apart        | 1.8–2.2 Hz via status topic | **Not formally established** |
| Bark recall                        | Not measured                   | ≥80%                        | **Not evaluated**            |
| False BARK rate on speech/noise    | Not measured                   | ≤10% provisional            | **Not evaluated**            |
| Event latency                      | Not measured                   | ≤1.5 s provisional          | **Not evaluated**            |
| Silence behavior                   | Not formally tested            | No events below gate        | **Not evaluated**            |

### Processing-Rate Qualification

The approximately 500 ms interval between these two event messages is **consistent with** a 2 Hz processing cadence.

It is not, however, sufficient by itself to establish the formal 2 Hz requirement. The test plan specifically distinguishes the continuously published `/bench/audio_classifier/status` topic from the event-driven `/audio/events` topic. Formal cadence verification therefore requires a sustained measurement of the status-topic rate rather than inference from two classification events.

### Discussion

The smoke test provides useful end-to-end evidence that the XVF3800, continuous audio pipeline, YAMNet model, and ROS classification interface are capable of responding to an actual Billie bark.

The first event had particularly high confidence at **0.992**, followed approximately half a second later by a `Dog` classification. This is a strong functional demonstration, but the formal DT-AUD-01 campaign requires a labeled dataset containing multiple barks, speech events, other noises, and silence in order to calculate recall, false-positive rate, and latency.

### Disposition

**SMOKE TEST SUCCESSFUL — FORMAL DT-AUD-01 INCOMPLETE.**

Live Billie bark classification has been demonstrated successfully. Formal classifier acceptance remains pending.

---

## 5.3 DT-AUD-02 — Direction of Arrival

### Test Objective

Measure XVF3800 DoA accuracy at known source bearings.

### Test Status

**DEFERRED.**

The current physical installation is located in the corner of a room and does not provide appropriate geometry for a controlled source-bearing test.

The planned DoA test requires a speaker to be positioned at known bearings around the array, ideally including:

* 0°
* 45°
* 90°
* 135°
* 180°
* 225°
* 270°
* 315°

A corner installation prevents this geometry from being exercised cleanly.

### Disposition

**DEFERRED — NO FAILURE ASSIGNED.**

DT-AUD-02 is explicitly optional in the current test plan and does not determine the pass/fail status of the audio classifier. It can be performed later when the array can be temporarily positioned in an open test area.

---

# 6. Pi Camera Module 3 NoIR

The sensor bench test plan defines:

* UT-NIR-01 — Image Acquisition and Real-Time Viewing
* UT-NIR-02 — Focus, Image Quality, and Low-Light/IR Sensitivity

No corresponding quantitative test results were supplied for inclusion in this report.

### Disposition

**NOT EVALUATED IN THIS REPORT.**

This should not be interpreted as either a pass or failure.

---

# 7. Cross-Sensor Findings

## 7.1 Acquisition Performance

The strongest result from the current campaign is the consistency of the sensor acquisition pipelines.

The OAK-D RGB and depth streams operated at approximately **4.97 Hz**, while the MLX90640 operated at **3.932 Hz** with extremely low period variation. Both are effectively at their commanded bench rates.

The XVF3800 also demonstrated correct **16 kHz** acquisition and exact **3.0 s** recording duration across the analyzed recordings. Its issue was not acquisition continuity but amplitude clipping during louder signals.

---

## 7.2 OAK-D Depth Capability

The OAK-D depth test demonstrated:

* approximately **4.3 cm median range error** at 1.492 m;
* approximately **1.2 cm robust standard deviation**;
* **96.7% valid pixels**;
* **99.6%** of valid depth samples within the BillieBot ±20 cm requirement.

These results provide substantial margin relative to the system-level ±0.20 m range requirement.

The remaining work is primarily procedural: repeat the measurement at the formally specified **2.00 m** distance and correct the plane-analysis startup-frame issue.

---

## 7.3 Thermal Capability

The MLX90640 demonstrated:

* excellent 4 Hz acquisition stability;
* 100% finite temperature data in UT-THM-01;
* a **21.8 °C** warm-target/background separation;
* very high background-referenced CNR;
* approximately **0.4 °C temporal noise**;
* less than 1 °C drift during the stationary test.

The outstanding thermal issues relate primarily to **test methodology and detector specificity** rather than basic thermal-camera operation:

1. the UT-THM-02 target reference temperature must be confirmed;
2. DT-THM-01 must establish a clean empty baseline;
3. DT-THM-01 should ultimately be repeated using Billie rather than a surrogate warm target.

---

## 7.4 Audio Capability

The XVF3800 successfully demonstrated:

* correct 16 kHz audio capture;
* correct recording duration;
* clean ambient acquisition;
* functioning YAMNet classification;
* successful live response to a Billie bark.

The principal acquisition issue is clipping during speech and impulse signals. Because clipping can degrade classification performance and waveform fidelity, UT-AUD-01 should be closed before relying on more extensive classifier characterization.

---

# 8. Recommended Follow-Up Actions

The following actions are recommended before formally closing the sensor bench campaign.

| Priority | Action                                                                                   | Reason                                                                                                   |
| -------- | ---------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| **1**    | Repeat UT-AUD-01 speech and impulse recordings after reducing clipping                   | Current clipping exceeds the required 0.1% limit.                                                        |
| **2**    | Confirm the actual UT-THM-02 target reference temperature                                | Determines whether the apparent +10.7 °C target bias is real or an incorrect ground-truth entry.         |
| **3**    | Repeat DT-THM-01 with a controlled empty baseline                                        | Current 7.84% baseline blob rate causes formal failure.                                                  |
| **4**    | Repeat DT-THM-01 with Billie at 0.5, 1.0, and 1.5 m                                      | The completed kettle run characterizes the algorithm but is not the specified Billie detector test.      |
| **5**    | Run UT-OAK-02 at exactly 2.00 m                                                          | Required for strict conformance to the specified acceptance condition.                                   |
| **6**    | Correct/re-run OAK-D plane-fit analysis while skipping invalid startup frames            | Enables the provisional ≤5 cm plane-RMSE criterion to be evaluated properly.                             |
| **7**    | Perform full DT-OAK-01 only if formal detector acceptance is required before integration | Current smoke test proves functionality but not controlled 1–4 m recall/false-positive performance.      |
| **8**    | Perform full DT-AUD-01 labeled-trial test                                                | Needed for formal bark recall, false-positive rate, latency, and sustained 2 Hz processing measurements. |
| **9**    | Perform DT-AUD-02 when the microphone array can be placed in an open test area           | Current corner configuration is unsuitable for controlled DoA characterization.                          |
| **10**   | Add UT-NIR-01/02 results when available                                                  | No NoIR results were included in the present dataset.                                                    |

---

# 9. Campaign Conclusion

The sensor bench campaign has established a strong functional baseline for the BillieBot perception and audio hardware.

The **OAK-D Lite** demonstrated stable approximately 5 Hz acquisition, valid RGB/depth/point-cloud outputs, strong stereo depth precision, and successful live detection of Billie. Its principal remaining work is formalization of the existing encouraging results at the exact required test conditions.

The **MLX90640** demonstrated highly stable approximately 4 Hz thermal acquisition and strong thermal contrast performance. Its raw-sensor behavior appears healthy. The remaining issues involve absolute-temperature reference documentation and false-positive behavior of the current global warm-pixel detector.

The **reSpeaker XVF3800** demonstrated functional multi-channel audio acquisition and successful live Billie bark classification. Formal acquisition acceptance remains open because speech and impulse recordings exceeded the clipping threshold.

The completed Type 2 smoke tests demonstrate that the major BillieBot perception paths are functioning end-to-end, but they should not be substituted for the controlled ground-truth datasets required for formal detector/classifier scoring.

Accordingly, the campaign can be summarized as:

* **OAK-D acquisition:** verified.
* **OAK-D depth capability:** verified with strong performance; exact 2 m formal point pending.
* **OAK-D dog detection:** functionally demonstrated; formal detector characterization pending.
* **MLX90640 acquisition:** verified.
* **MLX90640 thermal characterization:** strong; absolute target reference requires clarification.
* **Thermal blob detector:** positive detection capability demonstrated; false-positive/Billie-specific testing remains open.
* **XVF3800 acquisition:** functional, but clipping correction required for formal acceptance.
* **Audio classifier:** live bark detection demonstrated; formal statistical characterization pending.
* **XVF3800 DoA:** deferred.
* **Pi Camera 3 NoIR:** not evaluated in the supplied results.

Overall, the completed testing substantially reduces the technical risk associated with progressing to BillieBot multi-machine and integrated-system bringup, while clearly identifying the limited number of sensor-level items that remain to be formally closed.

---

*End of report.*
