# Fire robot Korean voice assets

The four PCM WAV files in this directory use the Korean Microsoft Neural
voice `ko-KR-SunHiNeural`. Runtime playback is offline; network access was
only required when the files were generated.

- `survivor_detected.wav`: survivor found and evacuation guidance
- `follow_me.wav`: follow-the-robot guidance
- `arrived_exit.wav`: safe-exit arrival guidance
- `emergency_stop.wav`: temporary emergency-stop guidance

The field launch first uses `~/fire_robot_audio`. If that directory is not
provisioned on a fresh Raspberry Pi, the installed copies in this package are
used automatically.
