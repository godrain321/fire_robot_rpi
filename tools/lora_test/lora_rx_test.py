#!/usr/bin/env python3
import argparse
import re


DEFAULT_BAUD = 9600
MESSAGE_PATTERN = re.compile(r'^HELLO_LORA,(\d+)$')


def parse_args():
    parser = argparse.ArgumentParser(
        description='Receive numbered test lines through an E220 serial connection.'
    )
    parser.add_argument('--port', required=True, help='Serial device, e.g. /dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD, help='UART baud rate')
    return parser.parse_args()


def parse_sequence(message):
    match = MESSAGE_PATTERN.fullmatch(message)
    return int(match.group(1)) if match else None


def main():
    args = parse_args()

    try:
        import serial
    except ImportError as exc:
        raise SystemExit(
            'pyserial is required. Install it with: python3 -m pip install pyserial'
        ) from exc

    expected_sequence = None

    try:
        with serial.Serial(args.port, args.baud, timeout=1) as connection:
            while True:
                raw_line = connection.readline()
                if not raw_line:
                    continue

                message = raw_line.decode('ascii', errors='replace').rstrip('\r\n')
                print(f'RX: {message}', flush=True)

                sequence = parse_sequence(message)
                if sequence is None:
                    continue

                if expected_sequence is not None and sequence != expected_sequence:
                    print(
                        'WARNING: sequence gap: '
                        f'expected {expected_sequence}, received {sequence}',
                        flush=True,
                    )
                expected_sequence = sequence + 1
    except KeyboardInterrupt:
        print('\nRX stopped.')
    except serial.SerialException as exc:
        raise SystemExit(f'Could not use serial port {args.port}: {exc}') from exc


if __name__ == '__main__':
    main()
