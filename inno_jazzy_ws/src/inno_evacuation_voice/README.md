# inno_evacuation_voice

독립 실행하는 오프라인 주기 대피 음성 패키지다. 기존 Mode 3/4 음성 노드를
수정하거나 포함하지 않는다.

안내 문장:

> 화재대피 로봇입니다. 출구로 안내해드리겠습니다.

기본 `drive_mode` 정책은 `/drive_mode=5` 진입 즉시 한 번 재생하고, 성공적인
재생 시작 시각을 기준으로 7초마다 반복한다. 다른 drive mode 또는
`/autonomy_cancel`에서는 즉시 중단한다. `always` 정책은 스피커 단독 시험용이다.

```bash
ros2 launch inno_evacuation_voice periodic_evacuation_voice.launch.py

ros2 launch inno_evacuation_voice periodic_evacuation_voice.launch.py \
  activation_mode:=always interval_sec:=7.0
```

`~/fire_robot_audio/evacuation_guide.wav`가 있으면 우선 사용하고, 없으면 설치된
패키지의 `share/inno_evacuation_voice/audio/evacuation_guide.wav`를 사용한다.
`audio_device:=auto`는 `/proc/asound/cards`의 첫 USB 카드만 선택하며 HDMI나 내장
오디오로 fallback하지 않는다.
