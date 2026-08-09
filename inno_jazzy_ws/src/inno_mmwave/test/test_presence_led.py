from inno_mmwave.presence_led import (
    DEFAULT_GPIO_LINES,
    DEFAULT_PHYSICAL_PINS,
    LgpioLedBank,
    PresenceLatch,
)


class FakeLgpio:
    def __init__(self):
        self.calls = []

    def gpiochip_open(self, chip):
        self.calls.append(('open', chip))
        return 99

    def gpio_claim_output(self, handle, line, level=0):
        self.calls.append(('claim', handle, line, level))

    def gpio_write(self, handle, line, level):
        self.calls.append(('write', handle, line, level))

    def gpio_free(self, handle, line):
        self.calls.append(('free', handle, line))

    def gpiochip_close(self, handle):
        self.calls.append(('close', handle))


def test_requested_bcm_to_physical_pin_order_is_fixed():
    assert DEFAULT_GPIO_LINES == (17, 27, 22, 23, 24)
    assert DEFAULT_PHYSICAL_PINS == (11, 13, 15, 16, 18)


def test_presence_latches_until_explicit_reset():
    latch = PresenceLatch()
    assert not latch.observe(False)
    assert latch.observe(True)
    assert latch.active
    assert not latch.observe(False)
    assert latch.active
    assert latch.reset()
    assert not latch.active


def test_rescue_trigger_can_reset_at_mission_end():
    latch = PresenceLatch()
    assert latch.observe(True, reset_on_false=True)
    assert latch.active
    assert latch.observe(False, reset_on_false=True)
    assert not latch.active


def test_active_high_bank_starts_and_closes_all_lines_low():
    fake = FakeLgpio()
    lines = (17, 27, 22)
    output = LgpioLedBank(4, lines, active_high=True, lgpio_module=fake)
    output.set_enabled(True)
    output.close()

    for line in lines:
        assert ('claim', 99, line, 0) in fake.calls
        assert ('write', 99, line, 1) in fake.calls
        assert ('write', 99, line, 0) in fake.calls
        assert ('free', 99, line) in fake.calls
    assert fake.calls[0] == ('open', 4)
    assert fake.calls[-1] == ('close', 99)


def test_active_low_bank_inverts_every_line():
    fake = FakeLgpio()
    lines = (17, 27)
    output = LgpioLedBank(4, lines, active_high=False, lgpio_module=fake)
    output.set_enabled(True)
    output.close()

    for line in lines:
        assert ('claim', 99, line, 1) in fake.calls
        assert ('write', 99, line, 0) in fake.calls
        assert ('write', 99, line, 1) in fake.calls


def test_duplicate_gpio_line_is_rejected_before_open():
    fake = FakeLgpio()
    try:
        LgpioLedBank(4, (17, 17), lgpio_module=fake)
    except ValueError as exc:
        assert 'duplicates' in str(exc)
    else:
        raise AssertionError('duplicate GPIO lines must be rejected')
    assert fake.calls == []
