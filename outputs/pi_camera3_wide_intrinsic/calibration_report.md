# Checkerboard Rational Polynomial 캘리브레이션 보고서

- 품질 상태: `warning`
- 모델: OpenCV Rational Polynomial 8계수 (단일 고정 모델)
- 해상도: 1280 × 720
- 체커보드: 8 × 9 내부 코너
- 한 칸 길이: 0.070000 m
- 검출 성공/입력: 38 / 41
- 개발 학습 RMS: 0.279535 px
- 검증 RMS: 0.266082 px
- 최종 전체 재학습 RMS: 0.275179 px

## 배포용 최종 파라미터

```text
[[825.579584263237   0.             647.367482082179]
 [  0.             824.195789699627 362.02541540697 ]
 [  0.               0.               1.            ]]
D = [36.3854434472, -31.8163399094, 0.000375656946725, 0.000896437062769, 108.870228301, 36.4962210106, -30.5402920072, 107.499612636]
order = [k1, k2, p1, p2, k3, k4, k5, k6]
```

`camera_info.yaml`은 위 최종 전체 재학습 K/D를 사용한다.
왜곡 보정 샘플의 new camera matrix는 배포용 원본 K를 대체하지 않는다.

## 경고

- Checkerboard observations are concentrated near the image center.
