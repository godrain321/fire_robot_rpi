from pathlib import Path
import wave

import pytest


ROOT = Path(__file__).resolve().parents[1]
AUDIO = ROOT / 'audio' / 'evacuation_guide.wav'


def test_packaged_audio_is_pcm_s16le_mono_24khz():
    if not AUDIO.is_file():
        pytest.skip('ko-KR-SunHiNeural evacuation_guide.wav was not generated')
    with wave.open(str(AUDIO), 'rb') as wav:
        assert wav.getcomptype() == 'NONE'
        assert wav.getsampwidth() == 2
        assert wav.getnchannels() == 1
        assert wav.getframerate() == 24000
        assert wav.getnframes() > 0
