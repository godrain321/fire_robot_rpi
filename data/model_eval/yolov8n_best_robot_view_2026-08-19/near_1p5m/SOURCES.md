# Approximate 1.5 m close-person visual check

The source photographs do not include measured camera-to-person distances. They
were selected by apparent subject size and low camera angle to approximate a
robot stopping about 1.5 m from a person. This is not a labeled benchmark.

Inference used `models/yolov8n_best.onnx`, image size 640, person class 0, and
the Mode 4 default confidence threshold 0.50.

| Test image | Source | Photographer |
| --- | --- | --- |
| `01_alley_back_view.jpg` | https://www.pexels.com/photo/woman-walking-on-narrow-old-street-4534041/ | Anas Jawed |
| `02_low_angle_walking.jpg` | https://www.pexels.com/photo/low-angle-shot-of-a-woman-in-fashionable-outfit-7682067/ | Mikhail Nilov |
| `03_crosswalk_walking.jpg` | https://www.pexels.com/photo/14743881/ | Pedro Colon |
| `04_extreme_low_angle.jpg` | https://www.pexels.com/photo/low-angle-photo-of-man-standing-near-light-pole-pointing-down-2874127/ | J Cruz |
| `05_dark_masked_walking.jpg` | https://www.pexels.com/photo/low-angle-shot-of-a-man-walking-on-the-sidewalk-10133565/ | Thom Gonzalez |
| `06_side_view_walking.jpg` | https://www.pexels.com/photo/unrecognizable-woman-walking-on-city-street-3860606/ | Darma Anggun Saputra |

Pexels license: https://www.pexels.com/license/
